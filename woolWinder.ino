#include "PID.h"

//define hardware pins using L298N motor driver
const int encA = 3;
const int dirPin = 5;
const uint8_t potPin = A0;

//need to define a rollover point so that we can carefully reset the target/current position so it doesnt overflow and cause the motor to reverse direction at max speed
const int maxEncoderCount = 30000;//16-bit

//define velocity variables
const int pulsesPerRotation = 4;
const int maxError = 255;
float velocityMax = 50;
//float velocity = 10; //rotation per second

//globals
volatile int encoderCount = 0;
PID pidController(dirPin);//pid controller instance
float prevTime, timeFloat, deltaT;//for timeKeeping
float target = 0;

void setup() {
  //define hardware pin directions
  pinMode(dirPin, OUTPUT);
  pinMode(encA, INPUT);
  pinMode(potPin, INPUT);

  //initialize the motor to be off, probably doesnt actually matter
  analogWrite(dirPin, 0);

  //set PID parameters that were found experimentally
  pidController.setParams(5, 2, 0, 255);

  attachInterrupt(digitalPinToInterrupt(encA), handleMotorEncoder, CHANGE);

  Serial.begin(9600);
}

void loop() {
  //timekeeping stuff
  unsigned long time = micros();
  timeFloat = time/1e6;//convert to seconds
  deltaT = timeFloat - prevTime;
  prevTime = timeFloat;

  //prevent any overflow problems
  if(encoderCount >= maxEncoderCount){
    encoderCount = 0;
    target = 0;
  }

  float vel = map(analogRead(potPin), 0, 1023, 0, velocityMax);

  //this function repeatedly sets a new position to the PID controller based on how much time has passed since the last time this function was ran and the desired velocity
  setTarget(vel, encoderCount, target);

}

void setTarget(float velocityIn, int currentSteps, float &target){
  float stepChange = velocityIn*deltaT*pulsesPerRotation;

  //only increment the target if the motor is able to keep up. prevents the error from indefinitely rising for no reason if the motor cant keep up for whatever reason
  if(target - encoderCount >= maxError){
    //Serial.println("Motor cant keep up!");
  }else{
    target = target+stepChange;
  }

  pidController.update(deltaT, target, currentSteps);
  pidController.setMotor(dirPin);

  Serial.print("Target:");
  Serial.print(target);
  Serial.print(",");
  Serial.print("Actual:");
  Serial.print(encoderCount);
  Serial.println();

}

void handleMotorEncoder(){
  encoderCount++;
}