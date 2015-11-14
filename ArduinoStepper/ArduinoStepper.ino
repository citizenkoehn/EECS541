////////////////////////////////////////////////////////////////////////
//
//	Arduino Stepper Motor Control Program
//
//	Controls stepper motor movement (collection of "transitions") via
//  keyframes inputted by the user.
//
//	Created on: 10/30/2105
//	Created by: Pete Koehn, Brandon Teh, and Jim Stanton
//	Last Modified on: 11/12/2015
//	Last Modified by: All
//
//  Test Frames
//  frameCount  position|swivelAngle|tiltAngle|duration
//  4|  000.000|00000|00000|000.000|  001.000|00000|00000|010.000|  001.500|00000|00000|018.000|  002.500|00000|00000|033.000|
//  4|  000.000|00000|00000|000.000|  002.000|00000|00000|010.000|  003.500|00000|00000|020.000|  007.500|00000|00000|050.000|
//
////////////////////////////////////////////////////////////////////////
#include <Stepper.h>
#include <Stream.h>
#define DEBUG 1
#define CALCTIME 1 
#define NumDigitsInt 5
#define NumDigitsFloat 7
#define NumDigitsKeyFrame ((2 * NumDigitsInt) + (2 * NumDigitsFloat) + 4) 
#define NumDigitsFrameCount 2

const int stepsPerRevolution = 200;  // Number of steps per revolution. Motor specification.
Stepper myStepper(stepsPerRevolution, 8,9,10,11);  // Output pins to H-Bridge Driver
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

void setup() {
  // Serial keyframe data input
  pinMode(dataPin, INPUT);
  
  // Transition initation input
  pinMode(startPin, INPUT);

  // Initialize the serial port:
  Serial.begin(9600);
}

void loop() {
    
  // Reset motor for next movement
//  if(startPin == HIGH){
//    start = HIGH;
//  }
  
  // Get keyframes
  int frameCount = 0; 
  if(start == HIGH) {
    Serial.print("Reading now\n");
    unsigned int frameCount = 0;  
    while(frameCount == 0) {
        if(Serial.available() == NumDigitsFrameCount){
          frameCount = Serial.parseInt();       
        }
    }    
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
         Serial.print("\n");
         #endif
		 
		 // Read Swivel Angle
         serialFrame.swivelAngle = Serial.parseInt();
		 #if DEBUG
         Serial.print("serialFrame.swivelAngle = ");
         Serial.println(serialFrame.swivelAngle);
         Serial.print("\n");
         #endif

		 // Read Tilt Angle
         serialFrame.tiltAngle = Serial.parseInt();
		 #if DEBUG
         Serial.print("serialFrame.tiltAngle = ");
         Serial.println(serialFrame.tiltAngle);
         Serial.print("\n");
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
    Serial.print(" Received all keyframes \n");
    #endif  
	
	// Example: Want 0.4 m in 8 seconds
	//    1 motor rotation = 0.1 m moved on track
	//    0.4 m = 4 rotations
	//    4 rotations / (8 seconds / (60 seconds/min)) = 30 rpm
	//    (0.1 m) / (200 steps per revolution) = 0.0005 m (on track) / motor step -- THIS IS DETERMINED BY MOTOR	
	//    0.4 m (on track) / 0.0005 m (movement on track / motor step) = 800 steps
	//    myStepper.setSpeed(30);
	//    myStepper.step(800);
  
	const float trackDistPerRotation = 0.1;    // Distance carriage moves on track per motor rotation
	const float trackDistPerStep = 0.1 / stepsPerRevolution;    // Distance carriage moves on track per motor step
	float numRotations = 0;    // Number of rotations required in transition
	int numSteps = 0;          // Number of steps required in transition
	int RPM = 0;               // RPMs required for transition
	
	#if CALCTIME
	unsigned long start = 0;
	unsigned long end = 0;
	unsigned long elapsed = 0;
	#endif
	
    // Loops through keyframes, execute transitions
	for(int j = 0; j < frameCount - 1; j++){
	  
	  #if CALCTIME
	  start = micros();
	  #endif
	  
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
      
	  #if CALCTIME
	  end = micros();
	  elapsed = end - start; // Resolution of 8 us
    Serial.println(elapsed);
	  #endif
	  
      // RPMs and the number of steps set for transition
      myStepper.setSpeed(RPM);  // Takes long
      myStepper.step(numSteps); // Takes int
    }
	
    // Done executing movement
    //start = LOW;
  }
}
