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
int start = 1;

// Keyframe structure
struct keyframe {
  float location;      // absolute position, left most = 0 and right-most = ???
  int swivelAngle;   // Like polar Phi
  int tiltAngle;     // Like polar Theta
  float time;          // absolute time
};

// Array of keyframes, dist in meters, time is absolute (in seconds)
struct keyframe keyframes[3] = {{.2,0,0,0},{.6,0,0,8},{1.4,0,0,13}};
  
void setup() {
  // transitiodn initated by user
  pinMode(startPin, INPUT);
  // argument for setSpeed is what needs to be modified. 0 for no movement.
  //myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  
  // Assumptions: 1 motor rotation = 0.1 m moved on track
  // Example: Want 0.4 m in 8 seconds
  //          0.4 m = 4 rotations
  //          4 rotations / (8 seconds / (60 seconds/min)) = 30 rpm
  //          (0.1 m) / (200 steps per revolution) = 0.0005 m (on track) / motor step -- THIS IS DETERMINED BY MOTOR
  //          0.4 m (on track) / 0.0005 m (movement on track / motor step) = 800 steps
  //          myStepper.setSpeed(30);
  //          myStepper.step(800);
  
    const float trackDistPerRotation = 0.1;
    const float trackDistPerStep = 0.1 / stepsPerRevolution;
    float numRotations = 0;
    int numSteps = 0;
    int RPM = 0;
    
    //start = HIGH;
    
    // Execute movement
    if (start == HIGH) {
      // Loop through keyframe array
      for(int j = 0; j < 2; j++){
        int k = j + 1;
        float transitionTime = keyframes[k].time - keyframes[j].time;
        Serial.print(transitionTime);
        Serial.print(" seconds\n");
        float movementDistance = keyframes[k].location - keyframes[j].location;
        Serial.print(movementDistance);
        Serial.print(" distance\n");
        numRotations = movementDistance / trackDistPerRotation;
        Serial.print(numRotations);
        Serial.print(" number of rotations\n");
        RPM = numRotations / (transitionTime / 60);
        Serial.print(RPM);
        Serial.print(" rpms\n");
        numSteps = movementDistance / trackDistPerStep;
        Serial.print(numSteps);
        Serial.print(" number of steps\n");
        
        myStepper.setSpeed(RPM);
        myStepper.step(numSteps);
        
        delay(500);
      }
      // Done executing movement
      start = LOW;
    }
}


//  // Batch of keyframes received
//  if start = digitalRead(startPin){
//    // Load keyframe data into local array
//    for(i = 0; i < 10; i++){
//      keyframes[i].location = ;
//      keyframes[i].swivelAngle = ;
//      keyframes[i].tiltAngle = ;
//      keyframes[i].time = ;
//    }
