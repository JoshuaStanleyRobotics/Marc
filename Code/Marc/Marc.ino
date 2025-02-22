/*
   Description: This program is used to control a Multi-directional Articulaed Rolling Cube robot with IMU feedback

        Wiring: The required components are 4x MG995 servos, a NRF24L01 radio module, and a Seeed XIAO nRF52840 Sense microcontroller (or suitable microcontroller and IMU replacement)
        The servos are connected with brown wires to GND, red wires to VCC, and orange wires to D0-D4
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D7, MOSI to D10, CE to D6, SCK to D8, MISO to D9
*/

//Libraries
#include "Servo.h"
Servo servo[4];

#include "SPI.h"
#include "RF24.h"
RF24 radio(6, 7);

#include "LSM6DS3.h"
LSM6DS3 IMU(I2C_MODE, 0x6A);
#include "Wire.h"

//Variables
byte data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int xDir = 0;
int yDir = 0;
int duration = 0;

//Constants
const byte chan[6] = "00007";                                                                       //Radio channel used
const int acute[] = {40, 144, 146, 45};                                                             //Calibration array defining the position of each servo at its minimum angle
const int obtuse[] = {141, 41, 48, 139};                                                            //Calibration array defining the position of each servo at its maximum angle

void setup() {
  Serial.begin(9600);                                                                               //Begin serial communication
  Serial.println("Serial Communication Initialized");

  radio.begin();                                                                                    //Begin radio communication
  radio.openReadingPipe(1, chan);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Radio Communication Initialized");

  IMU.begin();                                                                                      //Initialize Inertial Measurement Unit
  Serial.println("Inertial Measurement Unit Initialized");

  for (int i = 0; i < 4; i++) {                                                                     //Initialize servos
    servo[i].attach(i);
    servo[i].write(map(90, 45, 135, acute[i], obtuse[i]));
  }
  Serial.println("Servo Attached");

  delay(250);
  Serial.println("Running");
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));                                                                //Read in radio data if available

    if (data[1] != 0) {                                                                             //Forward and reverse controlled by front to back motion of left joystick
      if (data[1] > 127) {                                                                          //Rolling forward
        for (int i = 2; i < 4; i++) servo[i].write(acute[i]);
        delay(450);
        for (int i = 2; i < 4; i++) servo[i].write(obtuse[i]);
        delay(450);
      }
      else {                                                                                        //Rolling backward
        for (int i = 2; i < 4; i++) servo[i].write(obtuse[i]);
        delay(450);
        for (int i = 2; i < 4; i++) servo[i].write(acute[i]);
        delay(450);
      }
    }
    else if (data[0] != 0) {                                                                        //Side to side controlled by left to right motion of left joystick
      if (data[0] > 127) {                                                                          //Rolling right
        for (int i = 0; i < 2; i++) servo[i].write(acute[i]);
        delay(450);
        for (int i = 0; i < 2; i++) servo[i].write(obtuse[i]);
        delay(450);
      }
      else {                                                                                        //Rolling left
        for (int i = 0; i < 2; i++) servo[i].write(obtuse[i]);
        delay(450);
        for (int i = 0; i < 2; i++) servo[i].write(acute[i]);
        delay(450);
      }
    }
    else {                                                                                          //No motion is commanded
      for (int i = 0; i < 4; i++) servo[i].write((acute[i] + obtuse[i]) / 2);                       //Set the cube to its neutral shape
      delay(250);

      if (abs(IMU.readFloatAccelZ()) > 0.8) {                                                       //If the cube is on its front or back face, roll forward onto the top or bottom
        for (int i = 2; i < 4; i++) servo[i].write(acute[i]);
        delay(350);
      }
      else if (abs(IMU.readFloatAccelX()) > 0.8) {                                                  //If the cube is on its left or right face, roll sideways onto the top or bottom
        for (int i = 0; i < 2; i++) servo[i].write(acute[i]);
        delay(350);
      }
    }
    radio.flush_rx();
  }
  delay(5);
}
