# STM32F411 MAX31865 Temperature Sensor

This project implements a temperature measuring system using the STM32F411 microcontroller. It reads raw data from a PT100 sensor via the MAX31865 RTD-to-Digital converter using SPI interface. The data is processed, filtered, and converted to Celsius. The final result is transmitted to the PC via USB Virtual COM Port (CDC) every second (1 Hz).

## Hardware
* **Development Board:** WeAct STM32F411 "Black Pill"
* **RTD Converter:** MAX31865 Module
* **Sensor:** PT100 Thermoresistor (2-wire configuration)

## Software
* STM32CubeIDE 
