/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "command_responder.h"
#include <ArduinoBLE.h>
//custom uuid from GUID generator
#include "Arduino.h"

// Toggles the built-in LED every inference, and lights a colored LED depending
// on which word was detected.
BLEService service_pressure("19B10010-E8F2-537E-4F6C-D104768A1214");

BLEIntCharacteristic ble_threshold("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDescriptor ble_descriptor_threshold("1800", "second field whaat");
//Corresponds to SEND_MESSAGE in App GattAttributes
BLEIntCharacteristic ble_notify("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDescriptor ble_descriptor_notify("descriptor filed one", "descriptor field two");

BLECharacteristic ble_message("0001", BLERead | BLENotify, "declaration ble_message");



const int THRESH_1 = 720;
const int THRESH_2 = 900;
const int TIMER = 80;//time to hold the sensor
int counter_T1 = 0;
int counter_T2 = 0;
unsigned long delayStart = 0;
const int BUTTONONE = 4; // Naming switch button pin
int BUTTONstateONE = 0; // A variable to store Button Status / Input
const int BUTTONTWO = 6; // Naming switch button pin
int BUTTONstateTWO = 0; // A variable to store Button Status / 
boolean saidyes = false;

int countyes = 0;
///new
void RespondToCommand(tflite::ErrorReporter* error_reporter,
                      int32_t current_time, const char* found_command,
                      uint8_t score, bool is_new_command) {
  static bool is_initialized = false;
  if (!is_initialized) {
    pinMode(LED_BUILTIN, OUTPUT);
    // Pins for the built-in RGB LEDs on the Arduino Nano 33 BLE Sense
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    is_initialized = true;
  }
  static int32_t last_command_time = 0;
  static int count = 0;
  static int certainty = 220;

  if (is_new_command) {
    error_reporter->Report("Heard %s (%d) @%dms", found_command, score,
                           current_time);
    // If we hear a command, light up the appropriate LED.
    // Note: The RGB LEDs on the Arduino Nano 33 BLE
    // Sense are on when the pin is LOW, off when HIGH.
    if (found_command[0] == 'y') {
      last_command_time = current_time;
      saidyes = true;
      digitalWrite(LEDG, LOW);  // Green for yes
     
    }

    if (found_command[0] == 'n') {
      last_command_time = current_time;
      digitalWrite(LEDR, LOW);  // Red for no
    }

    if (found_command[0] == 'u') {
      last_command_time = current_time;
      digitalWrite(LEDB, LOW);  // Blue for unknown
    }
  }

  // If last_command_time is non-zero but was >3 seconds ago, zero it
  // and switch off the LED.
  if (last_command_time != 0) {
    if (last_command_time < (current_time - 3000)) {
      last_command_time = 0;
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, HIGH);
    }
    // If it is non-zero but <3 seconds ago, do nothing.
    return;
  }

  // Otherwise, toggle the LED every time an inference is performed.
  ++count;
  if (count & 1) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}


//bluetooth method

void bleLoop(){
   BUTTONstateONE = digitalRead(BUTTONONE);  // Reading button status / input
  BUTTONstateTWO = digitalRead(BUTTONTWO);  // Reading button status / input
  // poll for BLE events
  BLE.poll();
  int buttonValue = 0;// read thresholds
  ///////////////////have to filter here ////////////////////

  if(saidyes == true && countyes <10){
    countyes++;
    if(countyes == 8){
      saidyes = false;
      countyes =0;
    }
  }
  
  
 if (BUTTONstateONE == HIGH || saidyes == true){
    SendMessage((byte)1);
   // Serial.println("threshold");
  }
  else if(BUTTONstateTWO == HIGH){
 
     SendMessage((byte)2);
   // Serial.println("TWO");
  }
  else{
    SendMessage((byte)0);
   // Serial.println("0");
  }

}


void SendMessage(int buttonValue) {
 Serial.println("sentmessage");
  //boolean buttonPressed = ble_threshold.value() != buttonValue;
  // Serial.println(ble_threshold.value());
 // if (buttonPressed) {
 Serial.println(buttonValue);
    ble_threshold.writeValue(buttonValue);
    ble_notify.writeValue((byte)buttonValue);
    ble_message.writeValue("Button was pressed.");
//  }
}
void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  //  Serial.print("Connected event, central: ");
  // Serial.println(central.address());
}
void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  // Serial.print("Disconnected event, central: ");
  //  Serial.println(central.address());
}


void BLESetup(){

  delayStart=millis();
  Serial.begin(9600);
  pinMode (BUTTONONE, INPUT);
  pinMode (BUTTONTWO, INPUT);
  Serial.println("setup");

  
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("Silent_Guardians");
  //initialize the characteristics
  ble_threshold.setValue(0);
  ble_notify.setValue(0);
  ble_message.setValue("No pressure.");
  //add charact to service
   service_pressure.addCharacteristic(ble_threshold);
  service_pressure.addCharacteristic(ble_notify);
  service_pressure.addCharacteristic(ble_message);
  //add descriptor
  ble_threshold.addDescriptor(ble_descriptor_threshold);
  ble_threshold.addDescriptor(ble_descriptor_notify);
  //add service
  BLE.addService(service_pressure);

  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(service_pressure);

  // start advertising (so that device can connect)
  BLE.advertise();
  // Serial.println("Bluetooth device active, waiting for connections...");

  //event handler
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
}
