#include "Arduino.h"
#include "heltec.h"
#include <queue>

/*Code Written by: Jackson Butcher - University of Kentucky
  Description:
  This code handles LoRa Communication and Serial(USB) Communication with various devices.
  The overall goal of the code is to assist in RaspberryPi to RaspberryPi data transfer

  The Serial(USB) Communication is LoRa to RaspberryPi
  The LoRa Communication is LoRa to LoRa

  General Network Architecture
  RaspberryPi #1 --SERIAL(USB)--> LoRa Module #1 
  LoRa Module #1 --LoRaWAN--> LoRa Module #2
  LoRa Module #2 --Serial(USB)--> RaspberryPi #2
*/

//Global Variables, static and dynamic
#define BAND    915E6  //915Mhz Frequency for LoRa
#define PKT_SIZE 68 //bytes Packet size sent over architecture, shouldn't exceed 255 due to LoRa packet constraints.
#define INPUT_SERIAL_CACHE 256 //bytes
#define DELAY 1500 //1500 millseconds
#define SPREADING_FACTOR 7//Supported values are 6-12, 6 requires implicit header mode
#define SIGNAL_BANDWITH 250E3 //Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3,and 500E3.
#define TX_POWER 17 //dB
#define CODING_RATE 5 //denominator of the coding rate, supported values are 5-8
#define PREAMBLE 8 //Supported values are between 6 and 65535.


uint8_t lora_pkt_buffer[PKT_SIZE];
uint8_t serial_pkt_buffer[PKT_SIZE];
uint8_t ack[] = { 'P', 'a', 'c', 'k', 'e', 't', 'R', 'e', 'c', 'e', 'i', 'v', 'e', 'd', '\n' };
int8_t rssi = 0;
float snr = 0;
int bytesReadTotal = 0;

void setup() {
  /*Heltec Board Configuration for LoRa module, etc.
    NOTE: Heltec object instantiates Serial connection at baud rate of 115200 (bps) by default
    WARNING: modification of the default heltec.cpp file is needed to change the baud rate with this implementation
    WARNING: The Heltec object instantion automatically sends multiple serial statements,
    this leaves a high amount of junk data in the serial buffer
  */
  Heltec.begin(false /*Display*/, true /*LoRa*/, true /*Serial*/, true /*PABOOST*/, BAND /*BAND*/);
  // Heltec.display->init();
  // Heltec.display->setFont(ArialMT_Plain_16);
  // Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  // Heltec.display->clear();

  //Sets the software cache size of the receiving serial buffer(256 bytes by default), parameter is in bytes
  Serial.setRxBufferSize(INPUT_SERIAL_CACHE);

  //Note: DEFAULT parameters aren't default, must use the following functions to assure that LoRa is using the parameters you want to utilize
  //Transmission Speed was heavily hindered until realizing this fault in the library
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(SIGNAL_BANDWITH);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setPreambleLength(PREAMBLE);



  // Heltec.display->clear();
  // Heltec.display->drawString(0, 0, "Waiting... ");
  // Heltec.display->display();

  //Assigns to the LoRa object what function should run on Reception of a LoRa Packet
  //In this case the parameter is the function thats wanted to run
	LoRa.onReceive(onReceive);
  //Continous Receive Mode for LoRa
 	LoRa.receive();
}

//Input: Buffer length, amount to read from serial
//Output: String that holds the data read from the serial.
//Description:
// String SerialRead(int buf_length)
// {
//     uint8_t serial_buf[INPUT_SERIAL_CACHE];
//     size_t bytesRead = Serial.readBytes(serial_buf, buf_length);
    
//     String outString = "";
//     for(int i = 0;i<bytesRead;i++)
//       outString = outString + char(serial_buf[i]);
//     return outString;
// }

// size_t SerialReadNew(int buf_length)
// {
//     size_t bytesRead = Serial.readBytes(serial_pkt_buffer, buf_length);
//     return bytesRead;
// }

/*Input: A char buffer, an int that represents how much to read out of buffer
  Output: Nothing, Sends characters in buffer over Serial connection
  Description: This function does two things generalized.*/
void SerialWrite(uint8_t serial_packet[], int pkt_size)
{
  while(Serial.availableForWrite() < pkt_size)
    delay(10); //Wait until enough space to write packet over Serial
  Serial.write(serial_packet, pkt_size);
}


/*Input: ????
  Output: ????
  Description: Interrupt Function for reception of a LoRa pkt.*/
void onReceive(int packetSize)//LoRa receiver interrupt service
{
  //if (packetSize == 0) return;

  while (LoRa.available())
  {
    int bytes_read = LoRa.readBytes(lora_pkt_buffer, PKT_SIZE);
    if(bytes_read == PKT_SIZE)
    {
      SerialWrite(lora_pkt_buffer, bytes_read);
      rssi = LoRa.packetRssi();
      Serial.print(rssi);
      Serial.write(":RSSI:");
    }
  }

  //RssiDetection= abs(LoRa.packetRssi());   
}


//Transfer data across LoRa 
void LoRaTransfer(uint8_t packetTX[], size_t sizeTX)
{

  LoRa.beginPacket();

/*
 * LoRa.setTxPower(txPower,RFOUT_pin);
 * txPower -- 0 ~ 20
 * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
 *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
*/
  LoRa.setTxPower(TX_POWER, RF_PACONFIG_PASELECT_PABOOST);


  LoRa.write(packetTX , sizeTX);
  LoRa.endPacket();

  LoRa.receive();
}

//Main Loop for ESP32 within LoRa Module
void loop() {

  //Might need to do some alternative handling for onReceive
  //Could return to the original methodology where the LoRa is checked everytime through the loop.

  //Reads in how many bytes are waiting in serial cache
  int bytes_avail = Serial.available();

  if(bytes_avail > 0)
  { 
    //read data into temporary buffer
    uint8_t tempBuffer[bytes_avail];   
    int bytesRead = Serial.readBytes(tempBuffer , bytes_avail);  
    // appending data from temp buf to cache
    memcpy(serial_pkt_buffer + bytesReadTotal , tempBuffer, bytesRead);
    bytesReadTotal += bytesRead;
    //Once the full packet has been received then send
    if(bytesReadTotal >= PKT_SIZE)
    {
      //Transfer data
      LoRaTransfer(serial_pkt_buffer , bytesReadTotal);
      //clear cache
      memset(serial_pkt_buffer , 0 , bytesReadTotal);
      bytesReadTotal = 0;
      //send ack to RasPi, ready for another packet
      SerialWrite(ack, sizeof(ack));
    }
  }

}


