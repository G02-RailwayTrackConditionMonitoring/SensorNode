#include "NodeBLE.h"

 void NodeBLE::runBenchmark1(uint16_t packetSize,uint32_t numPackets){
        
        uint32_t len = packetSize*numPackets;
        //Prep the packet.
        uint8_t packet[len];
        for(uint32_t i = 0; i<len ;i++){
            
            
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
            
            //Start and end flags.
            if(i==0) packet[0] = 0xA5;
            if(i==numPackets-1) packet[0] = 0x5A;

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
            
            //Start and end flags.
            if(i==0) packet[0] = 0xA5;
            if(i==numPackets-1) packet[0] = 0x5A;
            
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
            
            //Start and end flags.
            if(i==0) {
                packet[0] = 0xA5;
            }
            else if(i==numPackets-1){
                 packet[0] = 0x5A;
            }
            
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