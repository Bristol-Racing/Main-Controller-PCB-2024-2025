/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-i2c-master-slave-arduino/
  ESP32 I2C Slave example: https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/examples/WireSlave/WireSlave.ino
*********/


#include "Wire.h"
#include "LoRa.h"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

//define the pins used by the transceiver module
#define ss 2
#define rst 5
#define dio0 17

int counter = 0;
char msg[30];

void onRequest() {
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
  Serial.println();
}

void onReceive(int len) {
  Serial.printf("onReceive[%d]: ", len);
  if (Wire.available()) {
    String received = "";
    received = Wire.readStringUntil('\n');

    Serial.print("Received message: ");
    Serial.println(received);
  
    //Send LoRa packet to receiver
    Serial.println("Sending to LoRa");
    LoRa.beginPacket();
    LoRa.print(received);
    LoRa.endPacket();
  }
  
}

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  // need to figure out pins
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //868E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(434E6)) { // 434E6
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.begin(115200);
  Serial.println("LoRa Initializing OK!");
  
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

  // sync word 0x39

/*#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
  Serial.print('Printing config %lu', i);
#endif*/

}

void loop() {
//  if (Wire.available()) {
//    String received = "";
//    received = Wire.readStringUntil('\n');
//  
//    Serial.print("Received message: ");
//    Serial.print(received);
//    Serial.println("Sending to LoRa");
//  
//    //Send LoRa packet to receiver
//    LoRa.beginPacket();
//    LoRa.print(msg);
//    LoRa.endPacket();
//  
//    counter++;
//  }
  delay(1000);
}
