//This is based off of the adafruit example: https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Peripheral/throughput/throughput.ino
#include "NodeBLE.h"
#include <functional>

    NodeBLE BLE_Stack;

    void NodeBLE::startBLE(String deviceName){

        Bluefruit.begin();  //This sets up the bluetooth software stack.

        Bluefruit.setName(deviceName.c_str());

        Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); //This sets the MTU, Event length, and some Queue sizes.

        Bluefruit.setTxPower(txPower);
        Bluefruit.Periph.setConnInterval(minConnInterval,maxConnInterval);
        
        Bluefruit.Periph.setDisconnectCallback(NodeBLE::disconnectedCallback);
        Bluefruit.Periph.setConnectCallback(NodeBLE::connectedCallback) ;
        
        //Setup the service and characteristics
        accelService = BLEService(accelService_UUID);
        err_t check = accelService.begin();   //This must be done before registering characteristics.
        if(check != ERROR_NONE){
            Serial.printf("Error setting up BLE Service: %d\n",check);
        }
        dataCharacteristic = BLECharacteristic(dataCharacteristic_UUID);
        dataCharacteristic.setPermission(SECMODE_OPEN,SECMODE_NO_ACCESS);
        dataCharacteristic.setFixedLen(61);
        dataCharacteristic.setProperties(CHR_PROPS_INDICATE);
        check = dataCharacteristic.begin(); //This will be registered to the last service to call begin().

        if(check != ERROR_NONE){
            Serial.printf("Error setting up BLE characteristic: %d\n",check);
        }
        }

    void NodeBLE::startAdvertising(){
        
        //Only be discoverable with BLE protocol not BT classic.
        Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE); 
        
        //For now let's just put the name. Later we should decide how we are identify our sensor nodes. (name, service uuid, etc.)
        Bluefruit.Advertising.addName(); 
        Bluefruit.Advertising.addUuid(dataCharacteristic_UUID);
        
        //This should be set to false later, to save power? We need to think about what happens if we lose BLE connection.
        //I belevive we can also save connections, like pairing/bonding.
        Bluefruit.Advertising.restartOnDisconnect(true); 

        
        Bluefruit.Advertising.setInterval(fastAdvInterval,slowAdvInterval);
        Bluefruit.Advertising.setFastTimeout(fastAdvTimeout);

        Bluefruit.Advertising.start(totalTimeout);
    }

     void NodeBLE::sendData(const void* data, uint16_t len){

         
         dataCharacteristic.indicate(data,len);
     }

     uint8_t NodeBLE::isConnected(){

         return Bluefruit.connected();
     }

    int8_t NodeBLE::getRSSI(){

        uint16_t connHandle = Bluefruit.connHandle();
        BLEConnection* connection = Bluefruit.Connection(connHandle);
        
        return connection->getRssi();
    }

    uint8_t NodeBLE::getPHY(){

        uint16_t connHandle = Bluefruit.connHandle();
        BLEConnection* connection = Bluefruit.Connection(connHandle);
        
        return connection->getPHY();
    }

    uint16_t NodeBLE::getMTU(){

        uint16_t connHandle = Bluefruit.connHandle();
        BLEConnection* connection = Bluefruit.Connection(connHandle);
        
        return connection->getMtu();
    }

    void NodeBLE::connectedCallback(uint16_t conn_handle){

        BLEConnection* conn = Bluefruit.Connection(conn_handle);
        Serial.println("Connected");

        // request PHY changed to 2MB
        Serial.println("Request to change PHY");
        conn->requestPHY(BLE_GAP_PHY_2MBPS);

        // request to update data length
        Serial.println("Request to change Data Length");
        conn->requestDataLengthUpdate();

        // request mtu exchange
        Serial.println("Request to change MTU");
        conn->requestMtuExchange(247);

        conn->monitorRssi();
        // request connection interval of 7.5 ms
        //conn->requestConnectionParameter(6); // in unit of 1.25

        // delay a bit for all the request to complete
        delay(1000);
        conn->requestMtuExchange(247);

        delay(1000);
    } 
    void NodeBLE::disconnectedCallback(uint16_t conn_handle, uint8_t reason){

        (void) conn_handle;
        (void) reason;

        Serial.println();
        Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
    }