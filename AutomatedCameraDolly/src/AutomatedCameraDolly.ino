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
//  *IMPORTANT* nrf8001 Bluetooth board only takes in 20 bytes of data in 1 chunk. *IMPORTANT*
//
//  Test Frames
//  frameCount  position (millimeters)|swivelAngle (degrees)|tiltAngle (degrees)|duration (milliseconds)
//  2|  000000|000|00|000000|  001000|030|20|005000|
//  4|  000000|000|00|000000|  001000|030|20|010000|  001500|045|25|018000|  002500|030|60|033000|
//  4|  000000|000|00|000000|  002000|000|00|010000|  003500|000|00|020000|  007500|000|00|050000|
//
////////////////////////////////////////////////////////////////////////
//
//  AccelStepper Library:
//  Copyright (C) 2009 Mike McCauley
//  $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
//
////////////////////////////////////////////////////////////////////////
// This version uses the internal data queing so you can treat it like Serial (kinda)!
#include <AccelStepper.h>
#include <SPI.h>
#include "Adafruit_BLE_UART.h"
//#include "GetKeyframes.h"
//#include "Calculations.h"
//#include "Execution.h"
#define DEBUG 1 // when DEBUG is on there is delay in keyframes
#define CALCTIME 1
#define NumDigitsInt 5
#define NumDigitsFloat 7
#define NumDigitsKeyFrame ((2 * NumDigitsInt) + (2 * NumDigitsFloat) + 4)
#define NumDigitsFrameCount 2

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
 
#define MAXSIZEKEYFRAME 21

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

// Define some steppers and the pins the will use
AccelStepper posStepper(AccelStepper::FULL4WIRE, 42, 40, 38, 36);
AccelStepper tiltStepper(AccelStepper::FULL4WIRE, 22, 24, 26, 44);
AccelStepper swivelStepper(AccelStepper::FULL4WIRE, 28, 30, 32, 34);
const int stepsPerRevolution = 200;  // Number of steps per revolution. Motor specification.

// Keyframe structure. Use int/long instead of float for faster processing? Measurements in mm instead of m?
struct keyframe {
  float location;    // 4 bytes. Absolute position, left (0) to right (???).
  int swivelAngle;   // 2 bytes. Like polar Phi.
  int tiltAngle;     // 2 bytes. Like polar Theta.
  float duration;    // 4 bytes. Absolute time.
};

int start = 1;

/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  Serial.begin(9600);
 
  Serial.println(F("PB&J Automated Camera Dolly System"));
  
  posStepper.setMaxSpeed(2000);
  tiltStepper.setMaxSpeed(2000);
  swivelStepper.setMaxSpeed(2000);
  
  BTLEserial.setDeviceName("DSLR"); /* 7 characters max! */

  BTLEserial.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started")); 
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    
   
  int frameCount = 0;
  if (start == HIGH) {
    Serial.println("Reading now!\n");
    unsigned int frameCount = 0;
    while (frameCount == 0) {
      BTLEserial.pollACI();
      //Serial.println("Polling ACI1"
      if (BTLEserial.available() >= NumDigitsFrameCount) { // watch for the == 
        frameCount = BTLEserial.parseInt();
      }
    }

    // Allocate keyframe structure
    struct keyframe *keyframes = (struct keyframe*) malloc(frameCount * sizeof(struct keyframe));

#if DEBUG
    Serial.print("read frameCount data\n");
#endif
    
    // Get keyframes
    int numReadFrames = 0;
    struct keyframe serialFrame;

    while (numReadFrames < frameCount) {
      BTLEserial.pollACI();
      if (BTLEserial.available() >= NumDigitsKeyFrame) {

        // Read Location
        serialFrame.location = BTLEserial.parseFloat();

        // Read Swivel Angle
        serialFrame.swivelAngle = BTLEserial.parseInt();

        // Read Tilt Angle
        serialFrame.tiltAngle = BTLEserial.parseInt();

        // Read Duration
        serialFrame.duration = BTLEserial.parseFloat();

//Statements to print location, swivelAngle, tiltAngle, and duration.
#if DEBUG
        Serial.print("serialFrame.location = ");
        Serial.println(serialFrame.location);
        
        Serial.print("serialFrame.swivelAngle = ");
        Serial.println(serialFrame.swivelAngle);
        
        Serial.print("serialFrame.tiltAngle = ");
        Serial.println(serialFrame.tiltAngle);
        
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
    //    (0.1 m) / (200 steps per revolution) = 0.0005 m (on track) / motor step -- THIS IS DETERMINED BY MOTOR
    //    0.4 m (on track) / 0.0005 m (movement on track / motor step) = 800 steps
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

    posStepper.setCurrentPosition(0);     // Resets position of stepper to zero
    tiltStepper.setCurrentPosition(0);    // Only once at beginning of movement?
    swivelStepper.setCurrentPosition(0);  // What if initial state is not zero (i.e. tilt of 45 deg)?

    // Loops through keyframes, execute transitions
    for (int j = 0; j < frameCount - 1; j++) {

      // Create index for "next" keyframe
      int k = j + 1;

      ////////////////////////////////////////////////////////////////
      // Position Calculations
      ////////////////////////////////////////////////////////////////
      // Distance covered during transition
      float locationDistance = keyframes[k].location - keyframes[j].location;

      // Number of motor steps taken during transition
      float numPosSteps = locationDistance / trackDistPerStep;

      ////////////////////////////////////////////////////////////////
      // Tilt Calculations
      ////////////////////////////////////////////////////////////////
      // Degrees tilted during transition
      float tiltDegrees = keyframes[k].tiltAngle - keyframes[j].tiltAngle;

      // Number of motor steps taken during transition
      float numTiltSteps = tiltDegrees / tiltDegPerStep;

      ////////////////////////////////////////////////////////////////
      // Swivel Calculations
      ////////////////////////////////////////////////////////////////
      // Degrees swiveled during transition
      float swivelDegrees = keyframes[k].swivelAngle - keyframes[j].swivelAngle;

      // Number of motor steps taken during transition
      float numSwivelSteps = swivelDegrees / swivelDegPerStep;

      ////////////////////////////////////////////////////////////////
      // Time Calculations
      ////////////////////////////////////////////////////////////////
      float transitionTime = keyframes[k].duration - keyframes[j].duration;

      posStepsPerSecond = numPosSteps / transitionTime;
      tiltStepsPerSecond = numTiltSteps / transitionTime;
      swivelStepsPerSecond = numSwivelSteps / transitionTime;
      
      /////////////////////////////////////////////////////////////////////////
      // EXECUTION
      /////////////////////////////////////////////////////////////////////////
      
      posStepper.move(numPosSteps);
      posStepper.setSpeed(posStepsPerSecond);
     
      tiltStepper.move(numTiltSteps);
      tiltStepper.setSpeed(tiltStepsPerSecond);
     
      swivelStepper.move(numSwivelSteps);
      swivelStepper.setSpeed(swivelStepsPerSecond);
      
      // Enable motors if utilized
      if(posStepper.distanceToGo() != 0){
        posStepper.enableOutputs();
      }
      if(posStepper.distanceToGo() != 0){
        swivelStepper.enableOutputs();
      }
      if(posStepper.distanceToGo() != 0){
        tiltStepper.enableOutputs();
      }
      // Run Motors
      while(posStepper.distanceToGo() != 0){
        posStepper.runSpeed();
        swivelStepper.runSpeed();
        tiltStepper.runSpeed();
      }

      posStepper.disableOutputs();
      swivelStepper.disableOutputs();
      tiltStepper.disableOutputs();
      
      /////////////////////////////////////////////////////////////////////////
      // DEBUG
      /////////////////////////////////////////////////////////////////////////
#if DEBUG
      Serial.println("**************************************");
      Serial.print("* Transition ");
      Serial.println(k);
      Serial.println("**************************************");
      Serial.print("Distance to move: ");
      Serial.println(locationDistance);
      
      Serial.print("Number of position steps: ");
      Serial.println(numPosSteps);
      
      Serial.print("Degrees to tilt: ");
      Serial.println(tiltDegrees);
      
      Serial.print("Number of tilt steps: ");
      Serial.println(numTiltSteps);
      
      Serial.print("Degrees to swivel: ");
      Serial.println(swivelDegrees);
      
      Serial.print("Number of swivel steps: ");
      Serial.println(numSwivelSteps);
      
      Serial.print("Seconds for movement: ");
      Serial.println(transitionTime);
      Serial.print("\n");
      
      //Execution
      Serial.print("Position (keyframes->location): ");
      Serial.println(keyframes[k].location);
      Serial.print("Position Steps/Second: ");
      Serial.println(posStepsPerSecond);
      
      Serial.print("Tilt Angle (keyframes->tiltAngle): ");
      Serial.println(keyframes[k].tiltAngle);
      Serial.print("Tilt Steps/Second: ");
      Serial.println(tiltStepsPerSecond);
      
      Serial.print("Swivel Angle (keyframes->swivelAngle): ");
      Serial.println(keyframes[k].swivelAngle);
      Serial.print("Swivel Steps/Second: ");
      Serial.println(swivelStepsPerSecond);
      Serial.print("\n");
#endif
    }
    
    free(keyframes); 
    }
  }
}
