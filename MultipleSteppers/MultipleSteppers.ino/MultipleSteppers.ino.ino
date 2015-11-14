// MultiStepper.pde
// -*- mode: C++ -*-
//
// Shows how to multiple simultaneous steppers
// Runs one stepper forwards and backwards, accelerating and decelerating
// at the limits. Runs other steppers at the same time
//
// Copyright (C) 2009 Mike McCauley
// $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
#define DEBUG 1
#define CALCTIME 1 
#define NumDigitsInt 5
#define NumDigitsFloat 7
#define NumDigitsKeyFrame ((2 * NumDigitsInt) + (2 * NumDigitsFloat) + 4) 
#define NumDigitsFrameCount 2

// Define some steppers and the pins the will use
AccelStepper posStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper tiltStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
AccelStepper swivelStepper(AccelStepper::FULL4WIRE, 10, 11, 12, 13);
const int stepsPerRevolution = 200;  // Number of steps per revolution. Motor specification.

// Keyframe structure. Use int/long instead of float for faster processing? Measurements in mm instead of m?
struct keyframe {
  float location;    // 4 bytes. Absolute position, left (0) to right (???).
  int swivelAngle;   // 2 bytes. Like polar Phi.
  int tiltAngle;     // 2 bytes. Like polar Theta.
  float duration;    // 4 bytes. Absolute time.
};

int start = 1;

void setup()
{  
  // Initialize the serial port:
  Serial.begin(9600);
}

void loop(){
  
  // Get keyframes
  int frameCount = 0; 
  if(start == HIGH){
    Serial.print("Reading now\n");
    unsigned int frameCount = 0;  
    while(frameCount == 0){
      if(Serial.available() == NumDigitsFrameCount){
        frameCount = Serial.parseInt();       
      }
    }

    // Allocate keyframe structure
    struct keyframe *keyframes = (struct keyframe*) malloc(frameCount * sizeof(struct keyframe));
    
    #if DEBUG
    Serial.print("read frameCount data\n");
    #endif
  
    int numReadFrames = 0;
    struct keyframe serialFrame;

    while(numReadFrames < frameCount){
      
      if(Serial.available() >= NumDigitsKeyFrame){ 
         
        // Read Location
        serialFrame.location = Serial.parseFloat();
        #if DEBUG
        Serial.print("serialFrame.location = ");
        Serial.println(serialFrame.location);
        //Serial.print("\n");
        #endif
        
        // Read Swivel Angle
        serialFrame.swivelAngle = Serial.parseInt();
        #if DEBUG
        Serial.print("serialFrame.swivelAngle = ");
        Serial.println(serialFrame.swivelAngle);
        //Serial.print("\n");
        #endif
        
        // Read Tilt Angle
        serialFrame.tiltAngle = Serial.parseInt();
        #if DEBUG
        Serial.print("serialFrame.tiltAngle = ");
        Serial.println(serialFrame.tiltAngle);
        //Serial.print("\n");
        #endif
        
        // Read Duration
        serialFrame.duration = Serial.parseFloat();
        #if DEBUG
        Serial.print("serialFrame.duration = ");         
        Serial.println(serialFrame.duration);
        Serial.print("\n");
        #endif
        
        keyframes[numReadFrames] = serialFrame;
        numReadFrames++;
      }
    }
    
    #if DEBUG
    Serial.println("Received all keyframes! \n");
    #endif

    ////////////////////////////////////////////////////////////////////
    // CALCULATIONS
    // Example: Want 0.4 m in 8 seconds
    //    1 motor rotation = 0.1 m moved on track
    //    0.4 m = 4 rotations
    //    4 rotations / (8 seconds / (60 seconds/min)) = 30 rpm
    //    (0.1 m) / (200 steps per revolution) = 0.0005 m (on track) / motor step -- THIS IS DETERMINED BY MOTOR  
    //    0.4 m (on track) / 0.0005 m (movement on track / motor step) = 800 steps
    //    myStepper.setSpeed(30);
    //    myStepper.step(800);
    ////////////////////////////////////////////////////////////////////
    
    const float trackDistPerRotation = 0.1;                     // Distance carriage moves on track per motor rotation
    const float trackDistPerStep = 0.1 / stepsPerRevolution;    // Distance carriage moves on track per motor step
    const float tiltDegPerStep = 1.8;     // Range 0 to 60 degrees? Tilt below 0 degrees?
    const float swivelDegPerStep = 1.8;   // -60 to 60 degrees with 0 straight ahead?
    float numRotations = 0;    // Number of rotations required in transition
    float posStepsPerSecond = 0;  // Number of steps per second of movement
    float tiltStepsPerSecond = 0;  // Number of steps per second of movement
    float swivelStepsPerSecond = 0;  // Number of steps per second of movement
    int numSteps = 0;          // Number of steps required in transition
    int RPM = 0;               // RPMs required for transition

    posStepper.setCurrentPosition(0);     // Resets position of stepper to zero
    tiltStepper.setCurrentPosition(0);    // Only once at beginning of movement?
    swivelStepper.setCurrentPosition(0);  // What if initial state is not zero (i.e. tilt of 45 deg)?

    // Loops through keyframes, execute transitions
    for(int j = 0; j < frameCount - 1; j++){
    
      // Create index for "next" keyframe
      int k = j + 1;

      ////////////////////////////////////////////////////////////////
      // Position Calculations
      ////////////////////////////////////////////////////////////////
      // Distance covered during transition
      float locationDistance = keyframes[k].location - keyframes[j].location;
      #if DEBUG
      Serial.print("Distance to move: ");
      Serial.println(locationDistance);
      #endif
      
      // Number of motor steps taken during transition
      float numPosSteps = locationDistance / trackDistPerStep;
      #if DEBUG
      Serial.print("Number of position steps\n");
      Serial.println(numPosSteps);
      #endif

      ////////////////////////////////////////////////////////////////
      // Tilt Calculations
      ////////////////////////////////////////////////////////////////
      // Degrees tilted during transition
      float tiltDegrees = keyframes[k].tiltAngle - keyframes[j].tiltAngle;
      #if DEBUG
      Serial.print("Degrees to tilt: ");
      Serial.println(tiltDegrees);
      #endif
      
      // Number of motor steps taken during transition
      float numTiltSteps = tiltDegrees / tiltDegPerStep;
      #if DEBUG
      Serial.print("Number of tilt steps: ");
      Serial.println(numTiltSteps);
      #endif
      
      ////////////////////////////////////////////////////////////////
      // Swivel Calculations
      ////////////////////////////////////////////////////////////////
      // Degrees swiveled during transition
      float swivelDegrees = keyframes[k].swivelAngle - keyframes[j].swivelAngle;
      #if DEBUG
      Serial.print("Degrees to swivel: ");
      Serial.println(swivelDegrees);
      #endif
      
      // Number of motor steps taken during transition
      float numSwivelSteps = swivelDegrees / swivelDegPerStep;
      #if DEBUG
      Serial.print("Number of swivel steps: ");
      Serial.println(numSwivelSteps);
      #endif
      
      ////////////////////////////////////////////////////////////////
      // Time Calculations
      ////////////////////////////////////////////////////////////////
      float transitionTime = keyframes[k].duration - keyframes[j].duration;
      #if DEBUG
      Serial.print("Seconds for movement: ");
      Serial.print(transitionTime);
      #endif

      posStepsPerSecond = numPosSteps/transitionTime;
      tiltStepsPerSecond = numTiltSteps/transitionTime;
      swivelStepsPerSecond = numSwivelSteps/transitionTime;
    }

    /////////////////////////////////////////////////////////////////////////
    // EXECUTION
    /////////////////////////////////////////////////////////////////////////
    posStepper.moveTo(keyframes->location);  // absolute position target. middle of track 0, ends are +/- 500 mm?
    posStepper.setSpeed(posStepsPerSecond);    // steps/second

    tiltStepper.moveTo(keyframes->tiltAngle);
    tiltStepper.setSpeed(tiltStepsPerSecond);
    
    swivelStepper.moveTo(keyframes->swivelAngle);
    swivelStepper.setSpeed(swivelStepsPerSecond);
    
    posStepper.run();
    tiltStepper.run();
    swivelStepper.run();
  }
}
