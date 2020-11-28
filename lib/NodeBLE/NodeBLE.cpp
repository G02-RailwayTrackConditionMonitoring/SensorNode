#include "NodeBLE.h"

        void NodeBLE::startBLE(){

            Bluefruit.begin();  //This sets up the bluetooth software stack.

            Bluefruit.setName(name.c_str());

            Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); //This sets the MTU, Event length, and some Queue sizes.

            Bluefruit.setTxPower(txPower);
            Bluefruit.Periph.setConnInterval(minConnInterval,maxConnInterval);

        }

        void NodeBLE::startAdvertising(){
            
            //Only be discoverable with BLE protocol not BT classic.
            Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE); 
            
            //For now let's just put the name. Later we should decide how we are identify our sensor nodes. (name, service uuid, etc.)
            Bluefruit.Advertising.addName(); 
            
            //This should be set to false later, to save power? We need to think about what happens if we lose BLE connection.
            //I belevive we can also save connections, like pairing/bonding.
            Bluefruit.Advertising.restartOnDisconnect(true); 

            
            Bluefruit.Advertising.setInterval(fastAdvInterval,slowAdvInterval);
            Bluefruit.Advertising.setFastTimeout(fastAdvTimeout);

            Bluefruit.Advertising.start(totalTimeout);
        }

        void connectedCallback(uint16_t conn_handle){} 
        void disconnetedCallback(uint16_t conn_handle, uint8_t reason){}