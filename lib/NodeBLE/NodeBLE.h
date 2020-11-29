#ifndef NODE_BLE_H
#define NODE_BLE_H

#include <bluefruit.h>


class NodeBLE{

    public:

        //Constructor
        NodeBLE()=default;
   
        void startBLE(String deviceName);
        void startAdvertising();
        bool sendData(const void* data, uint16_t len);
        uint8_t  isConnected();
        int8_t getRSSI();
        uint8_t getPHY();
        uint16_t getMTU();
        void runBenchmark1(uint16_t packetSize,uint32_t numPackets);
        void runBenchmark2(uint16_t packetSize,uint32_t numPackets);

    private:

        static void connectedCallback(uint16_t conn_handle);
        static void disconnectedCallback(uint16_t conn_handle, uint8_t reason);

        String  name{""};

        int8_t txPower{4}; // Can be the following values for NRF52840: -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8.
        
        uint16_t minConnInterval{6};    //In 1.25 ms units. So for example, a value a 6 coresponds to 6*1.25ms = 7.5 ms.
        uint16_t maxConnInterval{12};   //In 1.25 ms units.
        
        //How often we are sending advertising packets.
        //The BLE library allows for 2 different rates to be used. Fast advertising is done for fastAdvTimeout seconds and then the slow advertising rate is used for the remaining time until totalTimeout.
        uint16_t fastAdvInterval{32};   //In 0.625 ms units;
        uint16_t slowAdvInterval{244};  //In 0.625 ms units;
        uint16_t fastAdvTimeout{30};     //In seconds.
        uint16_t totalTimeout{60};      //In seconds?.
        
        //For defining the service and characteristic needed to send/receive data.
        //UUID generated from: https://www.guidgenerator.com/online-guid-generator.aspx
        const uint8_t accelService_UUID[16] = {0x7a,0xbd,0x7d,0x09,0xda,0xbd,0x4b,0x5d,0x88,0x2d,0x7f,0x4e,0x50,0x96,0xf8,0xf9};
        const uint8_t dataCharacteristic_UUID[16] = {0xe9,0xa4,0x19,0x3d,0x4d,0x05,0x45,0xf9,0x8b,0xc2,0x91,0x15,0x78,0x6c,0x96,0xc2};
        
        BLEService accelService; //Acceleration sensor service.
        BLECharacteristic dataCharacteristic;   
};


//The NodeBLE object should be used through this global object.
//This isn't the best practice, but since it makes sense to only ever have one NodeBLE object it's not that bad.
//This is done because there is an issue with passing member functions as callbacks :(
extern NodeBLE BLE_Stack; 

#endif