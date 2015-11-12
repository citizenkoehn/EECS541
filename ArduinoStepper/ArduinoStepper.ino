////////////////////////////////////////////////////////////////////////
//
//	Arduino Stepper Motor Control Program
//
//	Controls stepper motor movement (collection of "transitions")
//      via keyframes inputted by the user.
//
//	Created on: 10/30/2105
//	Created by: Pete Koehn, Brandon Teh, and Jim Stanton
//	Last Modified on: 11/12/2015
//	Last Modified by: All
//
// Test Frames
// frames             meters                        swiv                         tilt                         time
//   4z   000.000a00000a00000a000.000a  001.000b00000b00000b010.000  001.500c00000c00000c018.000  002.500d00000d00000d033.000d
//   4z   000.000a00000a00000a000.000a  002.000b00000b00000b010.000  003.500c00000c00000c020.000  007.500d00000d00000d050.000d
//
////////////////////////////////////////////////////////////////////////
#include <Stepper.h>
#include <Stream.h>
#define DEBUG 1
#define NumDigitsInt 5
#define NumDigitsFloat 7
#define NumDigitsKeyFrame ( (2 * NumDigitsInt) + (2 * NumDigitsFloat) + 4) 
#define NumDigitsFrameCount 2

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
//struct keyframe keyframesTest[3] = {{.2,0,0,0},{.6,0,0,8},{1.4,0,0,13}};
//struct keyframe keyframes[numTransitions];

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
  int frameCount = 0;  // probably delete this soon
  struct keyframe *serialKeyFrames = NULL;
  
  // Reset motor for next movement
  if(startPin == HIGH){
    start = HIGH;
  }
  
  // Flag allows movement execution
  if(start == HIGH) {
    Serial.print("Reading now\n");
    unsigned int frameCount = 0;  
    while(frameCount == 0) {
        if(Serial.available() == NumDigitsFrameCount){
          frameCount = Serial.parseInt();       
        }
    }    
    struct keyframe *keyFrames = (struct keyframe*) malloc(frameCount * sizeof(struct keyframe));
    
    Serial.print("read frameCount data\n");
    
    int numReadFrames = 0;
    struct keyframe serialFrame;

    while(numReadFrames < frameCount){
      
       if(Serial.available() >= NumDigitsKeyFrame){ 
         
         // Read location
         serialFrame.location = Serial.parseFloat();
         Serial.print("serialFrame.location = ");
         Serial.println(serialFrame.location);
         Serial.print("\n");

         

         serialFrame.swivelAngle = Serial.parseInt();

         Serial.print("serialFrame.swivelAngle = ");
         Serial.println(serialFrame.swivelAngle);
         Serial.print("\n");
         

         serialFrame.tiltAngle = Serial.parseInt();
         Serial.print("serialFrame.tiltAngle = ");
         Serial.println(serialFrame.tiltAngle);
         Serial.print("\n");
         

         // duration
         serialFrame.duration = Serial.parseFloat();
         
         Serial.print("serialFrame.duration = ");         
         Serial.println(serialFrame.duration);
         Serial.print("\n");
         
         keyFrames[numReadFrames] = serialFrame;
         numReadFrames++;  
       }
       
    }
    
    #if DEBUG
    Serial.print(" Received all keyframes \n");
    #endif  
    // Loops through keyframe array, executes each transition in movement
    for(int j = 0; j < frameCount-1; j++){
      
      // Create index for "next" keyframe
      int k = j + 1;
      
      // Time during transition
      float transitionTime = keyFrames[k].duration - keyFrames[j].duration;
      #if DEBUG
      Serial.print(transitionTime);
      Serial.print(" seconds\n");
      #endif
      
      // Distance covered during transition
      float movementDistance = keyFrames[k].location - keyFrames[j].location;
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
    //start = LOW;
   
  }
}
