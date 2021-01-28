# NodeBLE Library

This "library" provides an abstraction for the Bluetooth code required for the sensor node.

The main idea behind this is that there are a lot of parameters and setup for using BLE, so we will keep the main code clean by having all the BLE implementation in this library.

The underlying libraries used for Bluetooth communication is the _Adafruit Bluefruit Library_, which in turn uses the _Nordic SoftDevice_ BLE software stack. These are automatically included with the Platformio toolchain for the itsy bitsy. The bluefruit files are accessible and can be modified, for the Softdevice stack only the headers are available.

Here are links to the repositories for these dependencies:
* https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/libraries/Bluefruit52Lib
* https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/cores/nRF5/nordic/softdevice

## How To Use the Library

To use the library, include the header file "NodeBLE.h" in your file.

The library exposes a global BLE object, which you then use to call the BLE functions.

Here is a simple example:

```
#include <Arduino.h>
#include "NodeBLE.h"

void setup(){

    //Start the Bluetooth radio/stack and give our device the name "G02".
    BLE_Stack.startBLE("G02");
}

void loop(){

    if(!BLE_Stack.isConnected()){
        //Start advertising to allow a central device to discover and connect.
        BLE_Stack.startAdvertising();
    }
    else{
        //Run the benchmark once we have connected.
        BLE_Stack.startBenchmark4(2000);
    }
    delay(1000);

}
```

## Benchmarks
Here is an overview of the current benchmark functions.

- _benchmark2(uint16_t packetSize,uint32_t numPackets)_: This sends numPackets packets of size packetSize. If there is an error a packet will not be sent. If the packet size is larger then the current MTU, the data will be cutoff and not sent.
- _benchmark3(uint32_t numBytes)_: This sets the packet length to the current MTU. Then sends numBytes/MTU packets as fast as possible. If there is a software error transmitting, the code will attempt to send the data again.
- _benchmark4(uint32_t numBytes)_: This sets the packet length to the current MTU. Then sends numBytes/MTU packets, 2 at a time with a 5ms delay.If there is a software error transmitting, the code will attempt to send the data again.


## Connection Parameters

Here is an overview of the various parameters for the BLE connection:

- _name_: The device name that shows up during discovery.

- _txPower_: the current transmit power (in dB?). This ranges from -40 to 8, in certain steps.

- _mtu_: Maximum payload in bytes. The initial value is used to try and set the mtu and then it is updated to the actual value by the code.

- _minConnInterval/maxConnInterval_: The min/max connection interval (time slot given to a connection).

- _eventLength_: Another term for the connection interval apparently? Not sure how this plays with the min/max connection interval???

- _hvn_qsize_: Some kind of transmit queue size. Not sure how this works.

- _fastAdvIntervals_: The advertising interval during the fast phase.In 0.625ms units.

- slowAdvInterval: The advertising interval during the slow phase. In 0.625ms units.

- _fastAdvTimeout_: The length of the fast advertising phase. In seconds.

- _totalTimeout_: The total time spent advertising before stopping. In seconds.

- _accelService_UUID_: ID for our service. Can be used to identify the nodes.

- _dataCharacteristic_UUID_: ID for our characteristic which is used to send data. Central must know this same value!
