/**
 * Copyright 2020, George Spearing, UVM AERO
 * Data Acquisition Board CS5
 * (Rear DAQ w/ fan and brake control)
 */


#include <Arduino.h>
#include <mcp_can.h>

// CAN Settings
#define PIN_SPI_CAN_CS 5 // CAN SPI Chip
#define CAN_INT 2 // interrupt pin
MCP_CAN CAN(PIN_SPI_CAN_CS); // Set CS Pin
#define DAQ_CAN_INTERVAL 100 // time in ms
uint16_t lastSendDaqMessage = millis();

// CAN Address
#define ID_BASE 0x01
#define ID_REAR_DAQ_DATA ID_BASE + 1 // send data on this address
#define ID_DASH_RIGHT_DATA 0x74 // receive data from dash
#define ID_FRONT_PEDALBOARD 0x37 // receive data from pedal

// Output Pins
#define PIN_FAN_OUT 3
#define PIN_BRAKE_OUT 4

// intialize output values
bool brakeSig = false;
int fanSig;
bool autoTemp = false;

void filterCAN(unsigned long canID, unsigned char buf[8]){
  switch(canID){
    case ID_DASH_RIGHT_DATA:
      autoTemp = buf[2];
      // digitalWrite(PIN_FAN_OUT, autoTemp) // write fan output 'high' or 'low'
      break;
    case ID_FRONT_PEDALBOARD:
//      brakeSig = buf[5];
       digitalWrite(PIN_BRAKE_OUT, HIGH); // write brake output 'high' or 'low'
      break;
  }
}

void setup() {
  // setup PinMode

  pinMode(PIN_FAN_OUT, OUTPUT);
  pinMode(PIN_BRAKE_OUT, OUTPUT);

  // IF USING INTERRUPT PIN UNCOMMENT THIS CODE:
  // pinMode(CAN_INT, INPUT)

  // Initialize CANbus Interface
  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  CAN.setMode(MCP_NORMAL);

  Serial.begin(115200);
}

void loop() {

  // Initialize can buffers
  unsigned long id;
  unsigned char len = 0;
  unsigned char buf[8];

  // read the CANbus for data, filter as needed
  if (CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&id, &len, buf);
           digitalWrite(PIN_BRAKE_OUT,  ); // write brake output 'high' or 'low'

    Serial.print((uint8_t) buf);
    Serial.println();
    filterCAN(id, buf);
  }

}
