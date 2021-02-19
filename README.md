# Sensor Node

This repository contains the software running on the sensor nodes.

The project is setup for PlatfomIO, and targets the NRF52840.

Comment out line 27 (#define USE_BLE) to disable BLE for testing.

## Dependancies

A custom wrapper for the Adafruit BLE library is used.
See the README in lib/NodeBLE for more info about BLE code.

A modified version of the BorderFlight MPU-9250 library to make it compatible with the MPU-6500.
The current settings configure the IMU to max sampling rate (4kHz) and enables the 512 Byte buffer to store up to 85 3-axis Accelerometer samples. Overflow of the buffer will result in the oldest data being overwritten. After a specified collection interval, data is burst read by the processor and displayed on the serial monitor.
Seems to be working at 8MHz SPI read speeds, hardware should support up to 20MHz, but data stability issues are present at speeds exceeding this.
More info in lib/Bolder Flight Systems MPU9250/README.

The  SDFat library is used for interfacing with the SD card.
More info in lib/SDFat/README.

The CMSIS-DSP library is used for downsampling.
More info in lib/cmisis-dsp.

## Building the Project


This project is meant to be deployed on mulitple sensor node devices each with a unique name and identifiers.

In the platofmio.ini file, there must be a define that is unique for each specific sensor node.
As in either NODE_1, or NODE_2 etc must be defined when compiling the project.
This is used to choose between which BLE name and command UUID is compiled into the firmware.
So only this define must be changed, to switch  between firmware for each individual node.

The CFG_DEBUG = 2 build flag is currenlty enabled to print BLE debug info.

The BLE_GATT_ATT_MTU_DEFAULT should be changed from 23 to 247. 
This is in the file: .platformio\packages\framework-arduinoadafruitnrf52\cores\nRF5\nordic\softdevice\s140_nrf52_6.1.1_API\include\ble_gatt.h
This should be done manually and then clean and compile the project.

After this is done the compiled firmware can be built and upload  using the standard Platformio workflow.







