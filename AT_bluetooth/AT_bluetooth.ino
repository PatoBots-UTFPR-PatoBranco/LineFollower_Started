//Include the SoftwareSerial library
#include "SoftwareSerial.h"

//Create a new software  serial
SoftwareSerial bluetooth(10, 11); //TX, RX (Bluetooth)

#define KEY_PIN 9
#define LED 13

bool state = LOW;

void setup() {
  pinMode(KEY_PIN, OUTPUT);
  digitalWrite(KEY_PIN, HIGH);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, state);
  
  //Initialize the hardware serial
  Serial.begin(9600);
  Serial.println(F("Type the AT commands:"));

  //Initialize the software serial
  bluetooth.begin(38400);
}

void loop() {
  digitalWrite(LED, state);
  
  //Check received a byte from hardware serial
  if (Serial.available()) {
    char r = Serial.read(); //Read and save the byte
    bluetooth.print(r);  //Send the byte to bluetooth by software serial
    Serial.print(r);  //Echo
  }
  //Check received a byte from bluetooth by software serial
  if (bluetooth.available()) {
    char r = bluetooth.read(); //Read and save the byte
    Serial.print(r); //Print the byte to hardware serial

    state = !state;
  }
}
