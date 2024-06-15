#include "PID.h"
#include <Arduino.h>

//default constructor
PID::PID(int dirPinIn) : dirPin(dirPinIn), kP(1), kI(0), kD(0), ctlSig(0), errorPrev(0), errorInt(0), ctlSigMax(255) {}

//method to get the ctlSig
int PID::geterrorInt(){
  return static_cast<int>(errorInt);
}

int PID::getCtlSig(){
  return static_cast<int>(round(ctlSig));
}

//method to set the parameters
void PID::setParams(float kpIn, float kiIn, float kdIn, int ctlSigMaxIn){
  kP = kpIn; kI = kiIn; kD = kdIn; ctlSigMax = ctlSigMaxIn;
}

//written to work with L298N motor driver
void PID::setMotor(int dir1){
  //calculate power setting
  int pwr = round(ctlSig);

  //ensure we dont push the hardware past its limit
  if(pwr > ctlSigMax){
    pwr = ctlSigMax;
  }else if(pwr <= 20){
    pwr = 20;
  }

  //handle motor on/off
  if(dir1){
    analogWrite(dirPin, pwr);
  }else{
    analogWrite(dirPin, 0);
  }
}

//method to update the PID controller based on real time data
void PID::update(float deltaT, int desired, int actual){
  //calculate the error
  float error = desired - actual;

  //integral portion
  errorInt = errorInt + error*deltaT;
  
   
  //derivative
  float dedt = (error - errorPrev)/deltaT;

  ctlSig = kP*error + kI*errorInt + kD*dedt;
  
//store error for use in the derivative term
  errorPrev = error;


}

