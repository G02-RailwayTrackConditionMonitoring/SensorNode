//This is based off of the adafruit example: https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Peripheral/throughput/throughput.ino
#include "NodeBLE.h"
#include <functional>

    NodeBLE BLE_Stack;

    void NodeBLE::startBLE(String deviceName){

        Bluefruit.begin();  //This sets up the bluetooth software stack.

        Bluefruit.setName(deviceName.c_str());

        Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); //This sets the MTU, Event length, and some Queue sizes.
       Bluefruit.configPrphConn(247, 150, 3, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
       Bluefruit.configCentralConn(247, 150, 3, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
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
        dataCharacteristic.setTempMemory();
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

     bool NodeBLE::sendData(const void* data, uint16_t len){


         bool good  =  dataCharacteristic.notify(data,len);
        if(!good){

            Serial.println("Problem sending data!");
        }

        return good;
        //This freezes up. Could look into it later dont know if it is neccesssry.
        //  int16_t connHandle = Bluefruit.connHandle();
        //  BLEConnection* connection = Bluefruit.Connection(connHandle);
        //  connection->waitForIndicateConfirm();
         
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

    void NodeBLE::runBenchmark1(uint16_t packetSize,uint32_t numPackets){

        //Print out all the parameters for conveinience.
        uint16_t connHandle = Bluefruit.connHandle();
        BLEConnection* connection = Bluefruit.Connection(connHandle);
        uint16_t connInterval = connection->getConnectionInterval(); //1.25ms units
        connection->requestMtuExchange(236);

        Serial.println(" ");
        Serial.println("Starting BLE Benchmark:");
        Serial.printf("Packet Size: %d bytes\n",packetSize);
        Serial.printf("Total Data Sent: %d bytes\n",packetSize*numPackets);
        Serial.printf("PHY: %d Mbit\n",getPHY());
        Serial.printf("MTU: %d bytes\n",getMTU());
        Serial.printf("Initial RSSI: %d dB\n",getRSSI());
        Serial.printf("Connection interval: %d ms\n",connInterval*1.25);
        Serial.printf("Tx Power: %d\n",txPower);
        Serial.println(" ");

        
        //Prep the packet.
        uint8_t packet[packetSize*numPackets];
        for(uint32_t i = 0; i<packetSize*numPackets;i++){

            packet[i] = i%256;
        }
        uint32_t startTime = millis();

        sendData(packet,packetSize);

        uint32_t totalTime = (millis()-startTime);

        Serial.printf("Total Time: %d ms\n" , totalTime);
        Serial.printf("Total Thoughpout: %f bytes/second\n",(packetSize*numPackets)/(totalTime/1000.0));
        Serial.println("====================================");
    }

    void NodeBLE::runBenchmark2(uint16_t packetSize,uint32_t numPackets){

        //Print out all the parameters for conveinience.
        uint16_t connHandle = Bluefruit.connHandle();
        BLEConnection* connection = Bluefruit.Connection(connHandle);
        uint16_t connInterval = connection->getConnectionInterval(); //1.25ms units

        Serial.println(" ");
        Serial.println("Starting BLE Benchmark:");
        Serial.printf("Packet Size: %d bytes\n",packetSize);
        Serial.printf("Total Data Sent: %d bytes\n",packetSize*numPackets);
        Serial.printf("PHY: %d Mbit\n",getPHY());
        Serial.printf("MTU: %d bytes\n",getMTU());
        Serial.printf("Initial RSSI: %d dB\n",getRSSI());
        Serial.printf("Connection interval: %d ms\n",connInterval*1.25);
        Serial.printf("Tx Power: %d\n",txPower);
        Serial.println(" ");

        
        //Prep the pack, just copy a random number to the packet array.
        uint8_t packet[packetSize];
         memset(packet,97,packetSize);

        uint32_t startTime = millis();
        uint32_t pcktStart;

        uint32_t pcktTimes[numPackets];

        for(uint32_t i=0; i<numPackets; i++){
            
            pcktStart = millis();
            sendData(packet,packetSize);
            pcktTimes[i] = pcktStart-millis();
        }
        uint32_t totalTime = (millis()-startTime);

        // //Post Processing
        double minPacketTime = pcktTimes[0]/1000.0;
        double maxPacketTime = pcktTimes[0]/1000.0;
        double avgPacketTime = pcktTimes[0]/1000.0;

        for(uint32_t i=1; i<numPackets; i++){
            
            double curr = pcktTimes[i]/1000.0;
            avgPacketTime += curr;

            if(curr <minPacketTime){
                minPacketTime = curr;
            }

            if(curr > maxPacketTime){
                maxPacketTime = curr;
            }

         }


        Serial.printf("Total Time: %d ms\n" , totalTime);
        Serial.printf("Total Thoughpout: %f bytes/second\n",(packetSize*numPackets)/(totalTime/1000.0));
        Serial.printf("Avg Packet Time: %f seconds (%f bytes/sec)\n",avgPacketTime,avgPacketTime/packetSize);
        Serial.printf("Min Packet Time: %f seconds (%f bytes/sec)\n",minPacketTime,minPacketTime/packetSize);
        Serial.printf("Max Packet Time: %f seconds (%f bytes/sec)\n",maxPacketTime,maxPacketTime/packetSize);
        Serial.println("====================================");
    }