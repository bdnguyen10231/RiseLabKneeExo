#ifndef hlc_h
#define hlc_h

#include "hlc.h"
#include "llc.h"
#include "fsm.h"

class hlc {
  public:
  float controller(float ENC, float vENC, float IMU, float vIMU, float k, int angle, int spoint);
  int gravity = 0;
  int impedance = 0;
  int sinusoidal = 0;
  int manual = 0;
  float manRef = 0.0;
  float freq = 0;
  float amp = 0;
  
  private:
  float current_time = 0.0;
  float kneeAngle = 0.0;
  float hipAngle = 0.0;
  float kneeAngle_Vel = 0.0;
  float hipAngle_Vel = 0.0;

  float gravity_Gain = 1;
  float impedance_Gain = 1;

  float reference = 0.0;
  int maxTorque = 5;
};

#endif
