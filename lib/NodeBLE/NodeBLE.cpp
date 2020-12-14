//This is based off of the adafruit example: https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Peripheral/throughput/throughput.ino
#include "NodeBLE.h"
#include <functional>

    NodeBLE BLE_Stack;
    uint8_t NodeBLE::mtu = 247;

    void NodeBLE::startBLE(String deviceName){
        

        //Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); //This sets the MTU, Event length, and some Queue sizes automatically.

        //Not sure if these are configure the connection to the central or configure the connection as the central? So i've just included both for now.
        //TODO: Test which one is actually needed, I think we only need one of these.
        Bluefruit.configPrphConn(mtu, eventLength, hvn_qsize, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
        Bluefruit.configCentralConn(mtu, eventLength, hvn_qsize, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT); 

        Bluefruit.begin();  //This sets up the bluetooth software stack.

        Bluefruit.setName(deviceName.c_str());

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
        dataCharacteristic.setProperties(CHR_PROPS_NOTIFY);
        dataCharacteristic.setFixedLen(233);
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

        //  int16_t connHandle = Bluefruit.connHandle();
        //  BLEConnection* connection = Bluefruit.Connection(connHandle);
        //  connection->waitForIndicateConfirm();

        return good;
        //This freezes up. Could look into it later dont know if it is neccesssry.

         
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

        // uint16_t connHandle = Bluefruit.connHandle();
        // BLEConnection* connection = Bluefruit.Connection(connHandle);
        
        // return connection->getMtu();
        return mtu;
    }

     uint16_t NodeBLE::getConnInterval(){

                        
        uint16_t connHandle = Bluefruit.connHandle();
        BLEConnection* connection = Bluefruit.Connection(connHandle);
        uint16_t connInterval = connection->getConnectionInterval(); //1.25ms units

        return connInterval;
     }

    void NodeBLE::connectedCallback(uint16_t conn_handle){

        BLEConnection* conn = Bluefruit.Connection(conn_handle);
        Serial.println("Connected");

        // request PHY changed to 2MB
        Serial.println("Request to change PHY");
        conn->requestPHY(BLE_GAP_PHY_2MBPS);

        ble_gap_data_length_limitation_t limits;
         ble_gap_data_length_params_t dl_params;
        dl_params.max_rx_octets = 236;
        dl_params.max_tx_octets = 236;
        dl_params.max_rx_time_us = BLE_GAP_DATA_LENGTH_AUTO;
        dl_params.max_tx_time_us = BLE_GAP_DATA_LENGTH_AUTO;

        // request to update data length
        Serial.println("Request to change Data Length");
        conn->requestDataLengthUpdate(&dl_params,&limits);

        Serial.printf("DL exch, max_rx:%d ,lim:%d\n",dl_params.max_rx_octets,limits.rx_payload_limited_octets);
        Serial.printf("DL exch, max_tx:%d ,lim:%d\n",dl_params.max_tx_octets,limits.tx_payload_limited_octets);
        Serial.printf("DL exch, time:%d ,lim: %d us\n",dl_params.max_rx_time_us,limits.tx_rx_time_limited_us);

        delay(1000);
        // request mtu exchange, apparently this can only be done once? The NRF_ERROR_INVALID_STATE error will happen if we do this again?
        Serial.println("Request to change MTU");
        conn->requestMtuExchange(dl_params.max_rx_octets-limits.rx_payload_limited_octets);
        mtu = dl_params.max_rx_octets-limits.rx_payload_limited_octets;


        conn->monitorRssi();

        delay(1000);
    } 
    void NodeBLE::disconnectedCallback(uint16_t conn_handle, uint8_t reason){

        (void) conn_handle;
        (void) reason;

        Serial.println();
        Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
    }

    void NodeBLE::runBenchmark1(uint16_t packetSize,uint32_t numPackets){
        
        //Prep the packet.
        uint8_t packet[packetSize*numPackets];
        for(uint32_t i = 0; i<packetSize*numPackets;i++){

            packet[i] = i%256;
        }

        delay(1000);

        uint32_t startTime = millis();

        sendData(packet,packetSize);

        uint32_t totalTime = (millis()-startTime);

        Serial.println(" ");
        Serial.println("BLE Benchmark Results:");
        Serial.printf("Packet Size: %d bytes\n",packetSize);
        Serial.printf("Num packets: %d\n",numPackets);
        Serial.printf("Total Data Sent: %d bytes\n",packetSize*numPackets);
        Serial.printf("PHY: %d Mbit\n",getPHY());
        Serial.printf("MTU: %d bytes\n",getMTU());
        Serial.printf("Initial RSSI: %d dB\n",getRSSI());
        Serial.printf("Connection interval: %d ms\n",getConnInterval()*1.25);
        Serial.printf("Tx Power: %d\n",txPower);
        Serial.println(" ");
        Serial.printf("Total Time: %d ms\n" , totalTime);
        Serial.printf("Total Thoughpout: %f bytes/second\n",(packetSize*numPackets)/(totalTime/1000.0));
        Serial.println("====================================");
    }

    void NodeBLE::runBenchmark2(uint16_t packetSize,uint32_t numPackets){


        //Prep the pack, just copy a random number to the packet array.
        uint8_t packet[packetSize];
        memset(packet,97,packetSize);

        delay(1000);

        uint32_t startTime = millis();

        for(uint32_t i=0; i<numPackets; i++){
            
            sendData(packet,packetSize);
           
        }
        uint32_t totalTime = (millis()-startTime);

        Serial.println(" ");
        Serial.println("BLE Benchmark Results:");
        Serial.printf("Packet Size: %d bytes\n",packetSize);
        Serial.printf("Num packets: %d\n",numPackets);
        Serial.printf("Total Data Sent: %d bytes\n",packetSize*numPackets);
        Serial.printf("PHY: %d Mbit\n",getPHY());
        Serial.printf("MTU: %d bytes\n",getMTU());
        Serial.printf("Initial RSSI: %d dB\n",getRSSI());
        Serial.printf("Connection interval: %d ms\n",getConnInterval()*1.25);
        Serial.printf("Tx Power: %d\n",txPower);
        Serial.println(" ");
        Serial.printf("Total Time: %d ms\n" , totalTime);
        Serial.printf("Total Thoughpout: %f bytes/second\n",(packetSize*numPackets)/(totalTime/1000.0));
        Serial.println("====================================");
    }

    void NodeBLE::runBenchmark3(uint32_t numBytes){

        //Prep the pack, just copy a random number to the packet array.
        uint8_t packetSize = mtu;
        uint32_t numPackets = numBytes/(packetSize-3);
        uint8_t packet[packetSize];
         memset(packet,97,packetSize);

        delay(1000);

        uint32_t startTime = millis();
        
        bool err;
        for(uint32_t i=0; i<numPackets; i++){
            
            
            do{
            err = sendData(packet,packetSize);
            }
            while(!err);
           
        }
        uint32_t totalTime = (millis()-startTime);


        Serial.println(" ");
        Serial.println("BLE Benchmark Results:");
        Serial.printf("Packet Size: %d bytes\n",packetSize);
        Serial.printf("Num packets: %d\n",numPackets);
        Serial.printf("Total Data Sent: %d bytes\n",packetSize*numPackets);
        Serial.printf("PHY: %d Mbit\n",getPHY());
        Serial.printf("MTU: %d bytes\n",getMTU());
        Serial.printf("Initial RSSI: %d dB\n",getRSSI());
        Serial.printf("Connection interval: %d ms\n",getConnInterval()*1.25);
        Serial.printf("Tx Power: %d\n",txPower);
        Serial.println(" ");
        Serial.printf("Total Time: %d ms\n" , totalTime);
        Serial.printf("Total Thoughpout: %f bytes/second\n",(packetSize*numPackets)/(totalTime/1000.0));
        Serial.println("====================================");
    }

    void NodeBLE::runBenchmark4(uint32_t numBytes){

        //Prep the pack, just copy a random number to the packet array.
        uint8_t packetSize = mtu;
        uint32_t numPackets = numBytes/(packetSize-3);
        uint8_t packet[packetSize];
         memset(packet,97,packetSize);

        delay(1000);

        uint32_t startTime = millis();
        
        bool err;
        for(uint32_t i=0; i<numPackets; i++){
            
            
            do{
            err = sendData(packet,packetSize);
            }
            while(!err);
          if(numPackets%2 == 0) delay(5); //Send two packets every 5 ms.
        }
        uint32_t totalTime = (millis()-startTime);


        Serial.println(" ");
        Serial.println("BLE Benchmark Results:");
        Serial.printf("Packet Size: %d bytes\n",packetSize);
        Serial.printf("Num packets: %d\n",numPackets);
        Serial.printf("Total Data Sent: %d bytes\n",packetSize*numPackets);
        Serial.printf("PHY: %d Mbit\n",getPHY());
        Serial.printf("MTU: %d bytes\n",getMTU());
        Serial.printf("Initial RSSI: %d dB\n",getRSSI());
        Serial.printf("Connection interval: %d ms\n",getConnInterval()*1.25);
        Serial.printf("Tx Power: %d\n",txPower);
        Serial.println(" ");
        Serial.printf("Total Time: %d ms\n" , totalTime);
        Serial.printf("Total Thoughpout: %f bytes/second\n",(packetSize*numPackets)/(totalTime/1000.0));
        Serial.println("====================================");
    }