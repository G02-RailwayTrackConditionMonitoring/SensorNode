# Sensor Node

This repository contains the software running on the sensor nodes.

The project is setup for PlatfomIO, and targets the NRF52840.

This includes a modified version of the BorderFlight MPU-9250 library to make it compatible with the MPU-6500

The example initializes the IMU to max sampling rate (4kHz) and enables the 512 Byte buffer to store up to 85 3-axis Accelerometer samples. Overflow of the buffer will result in the oldest data being overwritten. After a specified collection interval, data is burst read by the processor and displayed on the serial monitor.

Currently is only functional at 1MHz SPI speeds, hardware should support up to 20MHz, but data stability issues are present at these speeds.
