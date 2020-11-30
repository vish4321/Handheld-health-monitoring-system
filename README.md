# Handheld-health-monitoring-system
Using STM32F4 Discovery board (STM32F407VGTx).
Intended to be run on Keil v5 IDE, with CMSIS and HAL add-ons.

The project also needs an MPU6050 inertial measurement unit and a MAX30100 pulse oximeter.
Pins 6,7 are SDA and SCL pins for I2C communication with the oximeter.
Pins 10,11 are SDA and SCL pins for I2C communication with the inertial measurement unit.
