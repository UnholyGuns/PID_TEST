#ifndef PID_H
#define PID_H

class PID{
  private:
    float kP, kI, kD, errorPrev, errorInt, ctlSig;
    int ctlSigMax;//calcualted control signal for the motor
    int dirPin;//hardware pin for the motor
  
  public:
    //constructor to initialize the parameters
    PID(int dirPinIn);

    //method to get the ctlSig
    int geterrorInt();

    int getCtlSig();

    float getDeltaT();

    //method to set the parameters
    void setParams(float kpIn, float kiIn, float kdIn, int ctlSigMaxIn);

    //written to work with L298N motor driver
    void setMotor(int dir1);

    //method to update the PID controller based on real time data
    void update(float deltaT, int desired, int actual);
};

#endif