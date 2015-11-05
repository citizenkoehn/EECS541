///////////////////////////////////////////////////////////////////
//
//	Arduino Stepper Motor Control Program
//
//	Controls stepper motor movement via keyframes inputted by
//      the user. Bluetooth to be implemented in the future.
//
//	Created on: 10/30/2105
//	Created by: Pete Koehn, Brandon Teh, and Jim Stanton
//	Last Modified on: 10/30/2015
//	Last Modified by: Pete Koehn
//
///////////////////////////////////////////////////////////////////
#include <Stepper.h>

const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 8,9,10,11);            
const int startPin = 12;
int start = 0;

// Keyframe structure
struct keyframe {
  int location;      // absolute position, left most = 0 and right-most = ???
  int swivelAngle;   // Like polar Phi
  int tiltAngle;     // Like polar Theta
  int time;          // absolute time
};

// Array of keyframes
keyframe keyframes[10];

void setup() {
  // transitiodn initated by user
  pinMode(startPin, INPUT);
  // argument for setSpeed is what needs to be modified. 0 for no movement.
  myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  
  // Batch of keyframes received
  if start = digitalRead(startPin){
    // Load keyframe data into local array
    for(i = 0; i < 10; i++){
      keyframes.i.location = ;
      keyframes.i.swivelAngle = ;
      keyframes.i.tiltAngle = ;
      keyframes.i.time = ;
    };
    // Execute movements
    for(j = 0; j < 9; j++){
      k = j + 1;
      int transitionTime = keyframes.k.time - keyframes.j.time;
    };
  };
  
}


/*
    if (start == HIGH) {
      // step one revolution in one direction:
      Serial.println("clockwise");
      myStepper.step(stepsPerRevolution);
*/
