Flight Computer V3 is the third version of model rocket flight computers I've been trying to build. It's designed to control a guided model rocket, and improve things from V2.
<br><br>Some specs about it are:
  - Input Voltage: 6-17V
  - Uses a STM32H743VIH6 microcontroller
  - 4 MOSFETs connected to battery input, I recommend use it for less than 3A
  - 5V power regulator that can regulate 5A using a TPS565208DDCR
  - Should have reverse polarity protection (NOT TESTED)
  - IMU: LSM6DSO32, which is 6 axis
  - Barometer: LPS22HB
  - Magnetometer: IIS2MDC
  - Flash Memory: W25Q256JVEIQ TR, which is 32MB
  - Programming methods: ST-LINK V2, FS USB-C
  - Onboard Indication: 2 addressable RGB LEDs
  - Communication outside board: SPI, I2C, FDCAN, USART
  - Has 8 servo connectors at 5V but total 5V max is 5A

<br><br>[Here is the link to the code in Arduino Framework.](FlightComputerV3_Setup)
<br>[Here is the link to the KiCad design files.](Flight_Computer_V2_Proto2)
