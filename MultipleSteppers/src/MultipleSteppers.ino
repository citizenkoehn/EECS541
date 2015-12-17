////////////////////////////////////////////////////////////////////////
//
//  Arduino Stepper Motor Control Program
//
//  Controls stepper motor movement (collection of "transitions") via
//  keyframes inputted by the user.
//
//  Created on: 10/30/2105
//  Created by: Pete Koehn, Brandon Teh, and Jim Stanton
//  Last Modified on: 11/12/2015
//  Last Modified by: All
//
//  Test Frames
//  frameCount  position|swivelAngle|tiltAngle|duration
//  2|  000.000|00000|00000|000.000|  001.000|00030|00020|005.000|
//  4|  000.000|00000|00000|000.000|  001.000|00030|00020|010.000|  001.500|00045|00025|018.000|  002.500|00030|00060|033.000|
//  4|  000.000|00000|00000|000.000|  002.000|00000|00000|010.000|  003.500|00000|00000|020.000|  007.500|00000|00000|050.000|
//
////////////////////////////////////////////////////////////////////////
//
//  AccelStepper Library:
//  Copyright (C) 2009 Mike McCauley
//  $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
//
////////////////////////////////////////////////////////////////////////
#include <AccelStepper.h>
#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include "GetKeyframes.h"
#include "Calculations.h"
#include "Execution.h"
#define DEBUG 0 // when DEBUG is on there is delay in keyframes
#define CALCTIME 1
#define NumDigitsInt 5
#define NumDigitsFloat 7
#define NumDigitsKeyFrame ((2 * NumDigitsInt) + (2 * NumDigitsFloat) + 4)
#define NumDigitsFrameCount 2
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

// Define some steppers and the pins the will use
//AccelStepper posStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
//AccelStepper tiltStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9, false);
//AccelStepper swivelStepper(AccelStepper::FULL4WIRE, 10, 11, 12, 13, false);
const int stepsPerRevolution = 200;  // Number of steps per revolution. Motor specification.

// Keyframe structure. Use int/long instead of float for faster processing? Measurements in mm instead of m?
struct keyframe {
  float location;    // 4 bytes. Absolute position, left (0) to right (???).
  int swivelAngle;   // 2 bytes. Like polar Phi.
  int tiltAngle;     // 2 bytes. Like polar Theta.
  float duration;    // 4 bytes. Absolute time.
};

int start = 0;

////////////////////////////////////////////////////////////////////////////
//  This function is called whenever select ACI events happen
////////////////////////////////////////////////////////////////////////////
void aciCallback(aci_evt_opcode_t event)
{
  switch(event)
  {
    case ACI_EVT_DEVICE_STARTED:
      Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      Serial.println(F("Connected!"));
      break;
    case ACI_EVT_DISCONNECTED:
      Serial.println(F("Disconnected or advertising timed out"));
      break;
    default:
      break;
  }
}
////////////////////////////////////////////////////////////////////////////
//  This function is called whenever data arrives on the RX channel
////////////////////////////////////////////////////////////////////////////
void rxCallback(uint8_t *buffer, uint8_t len)
{
  Serial.print(F("Received "));
  Serial.print(len);
  Serial.print(F(" bytes: "));
  for(int i=0; i<len; i++)
   Serial.print((char)buffer[i]); 

  Serial.print(F(" ["));

  for(int i=0; i<len; i++)
  {
    Serial.print(" 0x"); Serial.print((char)buffer[i], HEX); 
  }
  Serial.println(F(" ]"));

  /* Echo the same data back! */
  uart.write(buffer, len);
}

void setup()
{
  // Initialize the serial port:
  Serial.begin(9600);
//  posStepper.setMaxSpeed(2000);

  uart.setRXcallback(rxCallback);
  uart.setACIcallback(aciCallback);
  // uart.setDeviceName("NEWNAME"); /* 7 characters max! */
  uart.begin();
}

void loop() {
  
  uart.pollACI();
  
  // Get keyframes
  int frameCount = 0;
  if (start == HIGH) {
    Serial.println("Reading now!\n");
    unsigned int frameCount = 0;
    while (frameCount == 0) {
      if (Serial.available() == NumDigitsFrameCount) {
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

    while (numReadFrames < frameCount) {

      if (Serial.available() >= NumDigitsKeyFrame) {

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
    const float tiltDegPerStep = 0.5;     // Polar Theta. Half-degree resolution? Will depend on gear ratios
    const float tiltDegMax = 75.0;        // Range -20 to 75 degrees? Tilt below 0 degrees?
    const float tiltDegMin = -20.0;
    const float swivelDegPerStep = 0.5;   // Polar Phi: Half-degree resolution? Will depend on gear ratios
    const float swivelDegMax = 60.0;      // Range -60 to 60 degrees with 0 straight ahead?
    const float swivelDegMin = -60.0;
    float numRotations = 0;           // Number of rotations required in transition
    float posStepsPerSecond = 0;      // Number of steps per second of movement
    float tiltStepsPerSecond = 0;     // Number of steps per second of movement
    float swivelStepsPerSecond = 0;   // Number of steps per second of movement
    int numSteps = 0;                 // Number of steps required in transition
    int RPM = 0;                      // RPMs required for transition
    int delete_me_number_of_runs = 0;

//    posStepper.setCurrentPosition(0);     // Resets position of stepper to zero
//    tiltStepper.setCurrentPosition(0);    // Only once at beginning of movement?
//    swivelStepper.setCurrentPosition(0);  // What if initial state is not zero (i.e. tilt of 45 deg)?

    // Loops through keyframes, execute transitions
    for (int j = 0; j < frameCount - 1; j++) {

      // Create index for "next" keyframe
      int k = j + 1;

      ////////////////////////////////////////////////////////////////
      // Position Calculations
      ////////////////////////////////////////////////////////////////
      // Distance covered during transition
      float locationDistance = keyframes[k].location - keyframes[j].location;
#if DEBUG
      Serial.println("**************************************");
      Serial.print("* Transition ");
      Serial.println(k);
      Serial.println("**************************************");
      Serial.print("Distance to move: ");
      Serial.println(locationDistance);
#endif

      // Number of motor steps taken during transition
      float numPosSteps = locationDistance / trackDistPerStep;
#if DEBUG
      Serial.print("Number of position steps: ");
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
      Serial.println(transitionTime);
      Serial.print("\n");
#endif

      posStepsPerSecond = numPosSteps / transitionTime;
      tiltStepsPerSecond = numTiltSteps / transitionTime;
      swivelStepsPerSecond = numSwivelSteps / transitionTime;
      
      /////////////////////////////////////////////////////////////////////////
      // EXECUTION
      /////////////////////////////////////////////////////////////////////////    
#if DEBUG
      Serial.print("Position (keyframes->location): ");
      Serial.println(keyframes[k].location);
      Serial.print("Position Steps/Second: ");
      Serial.println(posStepsPerSecond);
#endif
//      posStepper.move(numPosSteps);
//      posStepper.setSpeed(posStepsPerSecond);
  
#if DEBUG
      Serial.print("Tilt Angle (keyframes->tiltAngle): ");
      Serial.println(keyframes[k].tiltAngle);
      Serial.print("Tilt Steps/Second: ");
      Serial.println(tiltStepsPerSecond);
#endif   
//      tiltStepper.move(numTiltSteps);
//      tiltStepper.setSpeed(tiltStepsPerSecond);
  
#if DEBUG
      Serial.print("Swivel Angle (keyframes->swivelAngle): ");
      Serial.println(keyframes[k].swivelAngle);
      Serial.print("Swivel Steps/Second: ");
      Serial.println(swivelStepsPerSecond);
      Serial.print("\n");
#endif   
//      swivelStepper.move(numSwivelSteps);
//      swivelStepper.setSpeed(swivelStepsPerSecond);
      
//      // Enable motors if utilized
//      if(posStepper.distanceToGo() != 0){
//        posStepper.enableOutputs();
//      }
//      if(posStepper.distanceToGo() != 0){
//        swivelStepper.enableOutputs();
//      }
//      if(posStepper.distanceToGo() != 0){
//        tiltStepper.enableOutputs();
//      }
//      // Run Motors
//      while(posStepper.distanceToGo() != 0){
//        posStepper.runSpeed(); // Execute transition
//  swivelStepper.runSpeed();
//        tiltStepper.runSpeed();
//      }
//      
//      posStepper.disableOutputs();
//      swivelStepper.disableOutputs();
//      tiltStepper.disableOutputs();
    }
    free(keyframes);
  }
}
