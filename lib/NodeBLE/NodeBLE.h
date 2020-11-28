#ifndef NODE_BLE_H
#define NODE_BLE_H

#include <bluefruit.h>

class NodeBLE{

    public:

        //Constructors
        NodeBLE()=default;
        NodeBLE(String deviceName) : name(deviceName) {}; 
        NodeBLE(String deviceName, int8_t transmitPower, uint16_t minConnInterval, uint16_t maxConnInterval) : 
                name(deviceName), txPower(transmitPower),minConnInterval(minConnInterval), maxConnInterval(maxConnInterval) {};

        void startBLE();
        void startAdvertising();
        
    private:

        void connectedCallback(uint16_t conn_handle);
        void disconnetedCallback(uint16_t conn_handle, uint8_t reason);

        String  name;

        int8_t txPower{4}; // Can be the following values for NRF52840: -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8.
        
        uint16_t minConnInterval{6};    //In 1.25 ms units. So for example, a value a 6 coresponds to 6*1.25ms = 7.5 ms.
        uint16_t maxConnInterval{12};   //In 1.25 ms units.
        
        //How often we are sending advertising packets.
        //The BLE library allows for 2 different rates to be used. Fast advertising is done for fastAdvTimeout seconds and then the slow advertising rate is used for the remaining time until totalTimeout.
        uint16_t fastAdvInterval{32};   //In 0.625 ms units;
        uint16_t slowAdvInterval{244};  //In 0.625 ms units;
        uint16_t fastAdvTimeout{30};     //In seconds.
        uint16_t totalTimeout{60};      //In seconds?.
        

};

#endif