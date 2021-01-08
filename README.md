# Sensor Node

This repository contains the software running on the sensor nodes.

The project is setup for PlatfomIO, and targets the NRF52840.
See the README in lib/NodeBLE for more info about BLE code.

In the platofmio.ini file, there must be a define that is unique for each specific sensor node.
As in either NODE_1, or NODE_2 etc must be defined when compiling the project.
This is used to choose between which BLE name and command UUID is compiled into the firmware.
So only this define must be changed, to switch  between firmware for each individual node.


Connect a button to Pin 9.

Currently the program will advertise over BLE when the SW button is pressed and will accept a connection.
When the button is pressed while conencted a benchmark will run and print the results over serial.

The CFG_DEBUG = 2 build flag is currenlty enabled to print BLE debug info.

The BLE_GATT_ATT_MTU_DEFAULT should be changed from 23 to 247. 
This is in the file: .platformio\packages\framework-arduinoadafruitnrf52\cores\nRF5\nordic\softdevice\s140_nrf52_6.1.1_API\include\ble_gatt.h
This should be done manually and then clean and compile the project.
