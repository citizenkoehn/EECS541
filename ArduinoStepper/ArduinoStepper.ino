////////////////////////////////////////////////////////////////////////
//
//	Arduino Stepper Motor Control Program
//
//	Controls stepper motor movement (collection of "transitions")
//      via keyframes inputted by the user.
//
//	Created on: 10/30/2105
//	Created by: Pete Koehn, Brandon Teh, and Jim Stanton
//	Last Modified on: 11/5/2015
//	Last Modified by: Pete Koehn
//
////////////////////////////////////////////////////////////////////////
#include <Stepper.h>
#define DEBUG 0

const int stepsPerRevolution = 200;  // Number of steps per revolution. Motor specification.
Stepper myStepper(stepsPerRevolution, 8,9,10,11);  // Output pins to H-Bridge Driver
const int numTransitions = 10;       // Number of transitions in movement
const int dataPin = 13;              // Pin for serial keyframe data input
const int startPin = 12;             // Pin indicating keyframes ready to be processed
int start = 1;                       // Flag to process keyframes

// Keyframe structure. Use int/long instead of float for faster processing? Measurements in mm instead of m?
struct keyframe {
  float location;    // 4 bytes. Absolute position, left (0) to right (???).
  int swivelAngle;   // 2 bytes. Like polar Phi.
  int tiltAngle;     // 2 bytes. Like polar Theta.
  float duration;    // 4 bytes. Absolute time.
};

// Array of keyframes: dist in meters, time is absolute (in seconds)
// Initialized here for testing. Will need to be read from user input
struct keyframe keyframesTest[3] = {{.2,0,0,0},{.6,0,0,8},{1.4,0,0,13}};
struct keyframe keyframes[numTransitions];

void setup() {
  // Serial keyframe data input
  pinMode(dataPin, INPUT);
  
  // Transition initated by user
  pinMode(startPin, INPUT);

  // Initialize the serial port:
  Serial.begin(9600);
}

void loop() {
    
  // Example: Want 0.4 m in 8 seconds
  //       1 motor rotation = 0.1 m moved on track
  //       0.4 m = 4 rotations
  //       4 rotations / (8 seconds / (60 seconds/min)) = 30 rpm
  //       (0.1 m) / (200 steps per revolution) = 0.0005 m (on track) / motor step -- THIS IS DETERMINED BY MOTOR
  //       0.4 m (on track) / 0.0005 m (movement on track / motor step) = 800 steps
  //       myStepper.setSpeed(30);
  //       myStepper.step(800);
  
  const float trackDistPerRotation = 0.1;    // Distance carriage moves on track per motor rotation
  const float trackDistPerStep = 0.1 / stepsPerRevolution;    // Distance carriage moves on track per motor step
  float numRotations = 0;    // Number of rotations required in transition
  int numSteps = 0;          // Number of steps required in transition
  int RPM = 0;               // RPMs required during transition
  
  // Reset motor for next movement
  if(startPin == HIGH){
    start = HIGH;
  }
    
  // Flag allows movement execution
  if(start == HIGH) {

    ///////////////////////////////////////////////
    // START HERE, need to read data in via Serial
    ///////////////////////////////////////////////
    // Initialize keyframe (movement) array
    for(int i = 0; i < numTransitions; i++){
      // Read four bytes of location data
      keyframes[i].location = Serial.read();
      // Read two bytes of swivel angle data
      keyframes[i].swivelAngle = Serial.read();
      // Read two bytes of tilt angle data
      keyframes[i].tiltAngle = Serial.read();
      // Read four bytes of time data
      keyframes[i].duration = Serial.read();
    }
        
    // Loops through keyframe array, executes each transition in movement
    for(int j = 0; j < 2; j++){
      
      // Create index for "next" keyframe
      int k = j + 1;
      
      // Time during transition
      float transitionTime = keyframes[k].duration - keyframes[j].duration;
      #if DEBUG
      Serial.print(transitionTime);
      Serial.print(" seconds\n");
      #endif
      
      // Distance covered during transition
      float movementDistance = keyframes[k].location - keyframes[j].location;
      #if DEBUG
      Serial.print(movementDistance);
      Serial.print(" distance\n");
      #endif
      
      // Number of rotations taken during transition
      numRotations = movementDistance / trackDistPerRotation;
      #if DEBUG
      Serial.print(numRotations);
      Serial.print(" number of rotations\n");
      #endif
     
      // RPMs of motor during transition
      RPM = numRotations / (transitionTime / 60);
      #if DEBUG
      Serial.print(RPM);
      Serial.print(" rpms\n");
      #endif
      
      // Number of motor steps taken during transition
      numSteps = movementDistance / trackDistPerStep;
      #if DEBUG
      Serial.print(numSteps);
      Serial.print(" number of steps\n");
      #endif
      
      // RPMs and the number of steps set for transition
      myStepper.setSpeed(RPM);  // Takes long
      myStepper.step(numSteps); // Takes int
    }
    
    // Done executing movement
    start = LOW;
  }
}
