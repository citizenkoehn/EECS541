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

struct keyframe {
  int location;
  int swivelAngle;
  int tiltAngle;
};

struct transition {
  keyframe keyframe1;
  keyframe keyframe2;
  int time; // seconds
};

void setup() {
  // transition initated by user
  pinMode(startPin, INPUT);
  // argument for setSpeed is what needs to be modified. 0 for no movement.
  myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  start = digitalRead(startPin);
  
  if (start == HIGH) {
    // step one revolution in one direction:
    Serial.println("clockwise");
    myStepper.step(stepsPerRevolution);
  };
}

void movement(struct transition trans1) {
  // Determine distance to travel between keyframes
  int x1 = trans1.keyframe1.location; 
  int x2 = trans1.keyframe2.location;
  int dist = x2 - x1;
  
  // move distance "dist" over time "trans1.time"
  
}

