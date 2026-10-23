/*
This code is for a custom STM32H743VIH6 PCB that uploads with DFU

This code can 

  - Read gyroscope and accelerometer data from a LSM6DSO32TR
  - Read pressure and temperature from a LPS22HB
  - calculate roll, pitch, and yaw using the Mahony filter
  - control 4 MOSFETs connected to the battery input
  - control 2 RGB addressable LEDs
  - control 8 servos being powered from 5v with a max of 5A
  - log to a micro SD card using 4 bit wide SDMMC, it cannot write to a SD card after its been taken out and put back in
*/
#include <Arduino.h>
#include <SPI.h>
#include <LSM6DSOSensor.h>
#include <STM32SD.h>
#include <elapsedMillis.h>
#include <LPS22HBSensor.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <SensorFusion.h>
#include <Mapf.h>

// change these variables ============================================
float localPressure = 30.30; // local pressure in inhg
int ledBrightness = 10; // change this from 0 to 100

String errors;
int angle = 0;

const unsigned long interval = 100; 
unsigned long previousMillis = 0;
bool ledON = false;

// Servos
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;
Servo Servo6;
Servo Servo7;
Servo Servo8;

#pragma region elapsedMillisTimers
elapsedMillis sdFlushTimer;
elapsedMillis sdStoreTimer;
elapsedMillis sinceStart;
elapsedMillis ledTimer;
elapsedMillis ServoTimer;
elapsedMillis eventDelayTimer;
elapsedMillis USBTimer;
elapsedMillis sdWait;
elapsedMillis sdInitCheck;
elapsedMillis servoSweep;
elapsedMillis errorPrintLimit;
#pragma endregion

// LSM6DSO (6 axis IMU)
#define SPI1_MOSI PB5
#define SPI1_MISO PB4
#define SPI1_SCK PB3
#define LSM6DSO_CS PA7

SPIClass dev_spi1(SPI1_MOSI, SPI1_MISO, SPI1_SCK);
LSM6DSOSensor AccGyr(&dev_spi1, LSM6DSO_CS);

// SD Card
#define SD_DETECT_PIN PE9

File logFile;
bool logToSD = true;
bool isSDIn = false;
bool SDError = true;

// magnetometer IIS2MDC

// flash

// sensor fusion
SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

// LPS22HB (pressure)
#define LPS22HB_CS PE3
LPS22HBSensor PressTemp(&dev_spi1, LPS22HB_CS);

// time since start
unsigned long timeSinceStart;

#pragma region LEDs
#define NUM_LEDS 2
#define LED_DATA_PIN ADDRESSABLE_LED
Adafruit_NeoPixel strip(NUM_LEDS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

unsigned long pixelPrevious = 0;        // Previous Pixel Millis
unsigned long patternPrevious = 0;      // Previous Pattern Millis
int patternCurrent = 0;       // Current Pattern Number
int patternInterval = 5000;   // Pattern Interval (ms)
bool patternComplete = false;

int pixelInterval = 50;       // Pixel Interval (ms)
int pixelQueue = 0;           // Pattern Pixel Queue
int pixelCycle = 0;           // Pattern Pixel Cycle
uint16_t pixelNumber = 2; 
#pragma endregion

// idk
#define TS_CAL1_ADDR ((uint16_t*)0x1FF1E820)
#define TS_CAL2_ADDR ((uint16_t*)0x1FF1E840) 

// Serial stuff
String noSerialLog;
bool isSerialConnected = false;
bool hasError = false;
// functions ========================================================================================

// initializes SD card
bool SDInit() {
  SD.begin();
  logFile = SD.open("logFile.csv", FILE_WRITE);
  // if the file is available, seek to last position
  if (logFile) {
    logFile.seek(logFile.size());

       
    if (logFile.size() > 0) {
        SerialUSB.print("File has data. Size in bytes: ");
        SerialUSB.println(logFile.size());
    } else {
        SerialUSB.println("File exists but it is empty.");
        logFile.print("time,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,pitch,roll,yaw,height,temp");
        logFile.println();
    }
  }
  // if the file isn't open, pop up an error:
  else {
    SerialUSB.println("error opening logFile.csv");
    return false;
  }
  SerialUSB.println("card initialized.");
  return true;
}

// calculates altitude from pressure sensor reading and 
float calculateAltitude(float pressure, float localPressure) {
  return (44330.0 * (1.0 - pow(pressure / localPressure, 1.0 / 5.255))) * 3.281;
}

void calculateAccelInG(int32_t acceleration[3]) {
  acceleration[0] /= 1000.0f;
  acceleration[1] /= 1000.0f;
  acceleration[2] /= 1000.0f;
}

float readInternalTemperature() {
    // 1. Read the factory calibration constants from system memory
    uint16_t ts_cal1 = *TS_CAL1_ADDR;
    uint16_t ts_cal2 = *TS_CAL2_ADDR;

    // 2. Read the raw ADC value from the internal temperature sensor channel
    // 'ATEMP' is the internal channel macro defined by STM32duino
    uint32_t raw_adc = analogRead(ATEMP);

    // 3. Compute temperature using linear interpolation formula
    // (110°C - 30°C) = 80
    float temperature = 30.0f + (80.0f * (float)(raw_adc - ts_cal1)) / (float)(ts_cal2 - ts_cal1);
    float inF = (temperature * (9/5)) + 32;
    return inF;
}

void flashLED(int flashTime, int whichLED, int colorR, int colorG, int colorB) {
  elapsedMillis ledFlash;

  while(ledFlash <= flashTime) {
    strip.setPixelColor(whichLED, strip.Color(colorR, colorG, colorB));
    strip.show();
  } 
  strip.setPixelColor(whichLED, strip.Color(0, 0, 0));
  strip.show();
  return;
}

void fireMOSFET(int whichPin, int onTime) {
  elapsedMillis onTimer;
  digitalWrite(whichPin, HIGH);
  if(onTime >= onTimer) {
    digitalWrite(whichPin, LOW);
    SerialUSB.println("========================================================================================");
    SerialUSB.println("Fired!");
    SerialUSB.println("========================================================================================");
    return;
  }
}

// setup ========================================================================================
void setup() {
  SerialUSB.begin(115200);
  analogReadResolution(16); 

  #pragma region switchPinMode
  // MOSFETs
  pinMode(SW1, OUTPUT);
  digitalWrite(SW1, LOW);

  pinMode(SW2, OUTPUT);
  digitalWrite(SW2, LOW);

  pinMode(SW3, OUTPUT);
  digitalWrite(SW3, LOW);

  pinMode(SW4, OUTPUT);
  digitalWrite(SW4, LOW);
#pragma endregion

  #pragma region servo
  //servo setup
  Servo1.attach(SERVO_1);
  Servo2.attach(SERVO_2);
  Servo3.attach(SERVO_3);
  Servo4.attach(SERVO_4);
  Servo5.attach(SERVO_5);
  Servo6.attach(SERVO_6);
  Servo7.attach(SERVO_7);
  Servo8.attach(SERVO_8);
  #pragma endregion


  // LEDs
  strip.begin();           
  strip.setBrightness(map(ledBrightness, 0, 100, 0, 255));
  strip.show(); 

  // sensor begin
  AccGyr.begin();
  AccGyr.Enable_X();  
  AccGyr.Enable_G();
  AccGyr.Set_G_FS(500);
  AccGyr.Set_X_FS(16);
  AccGyr.Set_G_ODR(500);
  AccGyr.Set_X_ODR(500);

  PressTemp.begin();
  PressTemp.Enable();
  localPressure *= 33.8639;

  #pragma region SDCard
  // SD card
  SD.setDx(SDMMC1_D0, SDMMC1_D1, SDMMC1_D2, SDMMC1_D3);
  SD.setCMD(SDMMC1_CMD);
  SD.setCK(SDMMC1_CK);
  
  if (digitalRead(SD_DETECT_PIN) == LOW) {
    isSDIn = true;
    SDInit();
  } else {
    isSDIn = false;
  }
  isSDIn = true;
  delay(100);

  if(isSDIn == true) {
    SDInit();
  }
  #pragma endregion
}

// loop ========================================================================================
void loop() {
  //lsm6dso 
  int32_t acceleration[3];
  int32_t gyroscope[3];
  AccGyr.Get_X_Axes(acceleration);  
  AccGyr.Get_G_Axes(gyroscope);

  //lps22hb
  float pressure;
  float temperature;
  PressTemp.GetPressure(&pressure);  
  PressTemp.GetTemperature(&temperature);

  timeSinceStart = millis();

  float mcu_temp = readInternalTemperature();

  gx = gyroscope[0] / 1000.0f * DEG_TO_RAD;  gy = gyroscope[1] / 1000.0f * DEG_TO_RAD;  gz = gyroscope[2] / 1000.0f * DEG_TO_RAD;
  ax = acceleration[0];  ay = acceleration[1];  az = acceleration[2];
  
  calculateAccelInG(acceleration);
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  

  // calculates pitch roll and yaw with the Mahony filter
  pitch = fusion.getPitch();
  roll = fusion.getRoll();   
  yaw = fusion.getYaw();

  // prints over USB every 1 second
  if(USBTimer >= 1000 && SerialUSB) {
    USBTimer = 0;
    isSerialConnected = true;

    SerialUSB.println(noSerialLog);
    noSerialLog = "";
    SerialUSB.print("Pitch:\t"); SerialUSB.println(pitch);
    SerialUSB.print("Roll:\t"); SerialUSB.println(roll);
    SerialUSB.print("Yaw:\t"); SerialUSB.println(yaw);
    SerialUSB.print("MCU Temp\t");  SerialUSB.println(mcu_temp, 2);
    SerialUSB.print("Ambient Temp in F\t"); SerialUSB.println((temperature * (9.0/5.0)) + 32);
    SerialUSB.print("Altitude in ft\t");  SerialUSB.println(calculateAltitude(pressure, localPressure));
    Serial.println();

    
    if(SDError == true) {
      SerialUSB.println("error on logFile.csv file handle");
    }
    SerialUSB.println();
  }

  // checks and writes to micro SD card every 2 ms
  if(sdStoreTimer >= 2) {
    sdStoreTimer = 0;
    strip.show();
    //writing to sd card
    if (logFile && isSDIn == true) {
      logFile.print(timeSinceStart);  logFile.print(",");
      logFile.print(acceleration[0]);  logFile.print(",");
      logFile.print(acceleration[1]);  logFile.print(",");
      logFile.print(acceleration[2]);  logFile.print(",");
      logFile.print(gyroscope[0]);  logFile.print(",");
      logFile.print(gyroscope[1]);  logFile.print(",");
      logFile.print(gyroscope[2]);  logFile.print(",");
      logFile.print(pitch); logFile.print(",");
      logFile.print(roll);  logFile.print(",");
      logFile.print(yaw); logFile.print(",");
      logFile.print(calculateAltitude(pressure, localPressure));  logFile.print(",");
      logFile.println(temperature);
      SDError = false;
    }
    else {
      SDError = true;
      hasError = true;
    }
  }

  isSDIn = (digitalRead(SD_DETECT_PIN) == LOW);
  // logging to sd card every 0.4 seconds
  if(sdFlushTimer >= 400) {
    sdFlushTimer = 0;
    logFile.flush();
  }

  // SD initialization check
  if(sdInitCheck >= 2000) {
    sdInitCheck = 0;
    if(isSDIn == false) {
      if(SDInit() == false) {
        isSDIn = false;
        strip.setPixelColor(0, strip.Color(100, 0, 0));
        // strip.show();
      } else {
        isSDIn = true;
        strip.setPixelColor(0, strip.Color(0, 100, 0));
        // strip.show();
      }
    }
  }
  
  if(hasError == true) {
    // checks if there is sd error 
    if(SDError == true && errorPrintLimit >= 10) {
      errorPrintLimit = 0;
      if(!SerialUSB) {
        noSerialLog += timeSinceStart;
        noSerialLog += " error on logFile.csv file handle\n";
      } else {

      }
    }
  }

  int ledR = mapf(pitch, 0, 180, 0, 255);
  int ledG = mapf(roll, 0, 180, 0, 255);
  int ledB = mapf(yaw, 0, 180, 0, 255);

  if(ledTimer >= 50 && ledON == true) {
    strip.setPixelColor(0, strip.Color(ledR, ledG, ledB));
    strip.show();
  }

  if(ServoTimer >= 5) {
    Servo1.write(pitch);
  }
  
  if (angle < 180 && servoSweep >= 5) {
    servoSweep = 0;
    angle++;       
    Servo2.write(angle);
  }
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
     
    angle += 1;
    if(angle >= 180) {
      angle = 0;
    }
    
    Servo2.write(angle);
  }
  
  if (Serial.available() > 0) {
    String command = SerialUSB.readStringUntil('\n');
    command.trim();
    
    if(command.equalsIgnoreCase("LEDOFF")) {
      ledON = false;
    }
    if(command.equalsIgnoreCase("LEDON")) {
      ledON = true;
    }

    if(command.equalsIgnoreCase("FIRE SW1")) {
      
      strip.setBrightness(255);
      flashLED(500, 0, 255, 0, 0);
      fireMOSFET(SW1, 2000);
    }

    if(command.equalsIgnoreCase("FIRE SW2")) {
      
      strip.setBrightness(255);
      flashLED(500, 0, 0, 255, 0);
      fireMOSFET(SW2, 2000);
    }

    if(command.equalsIgnoreCase("FIRE SW3")) {
      
      strip.setBrightness(255);
      flashLED(500, 0, 0, 0, 255);
      fireMOSFET(SW3, 2000);
    }

    if(command.equalsIgnoreCase("FIRE SW4")) {
      
      strip.setBrightness(255);
      flashLED(500, 0, 255, 255, 255);
      fireMOSFET(SW4, 2000);
    }
  }
}


