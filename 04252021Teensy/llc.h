#ifndef llc_h
#define llc_h

#include "llc.h"
#include "arduino.h"

class llc {
  public:
  float measurement = 0.0;
  float signalx[3] = {0.0, 0.0, 0.0}; // {current, last, last last}
  float current = 0.0;

  float maxBackLash = 0.0;
  float minBackLash = 0.0;

  void controller(float reference, float rawAngle);
  int paused = 0;

  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float Kf = 0.0;
  float error[3] = {0.0, 0.0, 0.0};
  float bound = 5.0;

  float a[3] = {0.0, 0.0, 0.0}; //{a1, a2}
  float b[3] = {0.0, 0.0, 0.0}; //{b1, b2}
  float N = 3;

  long lowPass_onePole(float, float, float);
  float errorDot_F = 0.0;

  signed int motorDirection = 1;
   
  float rawAngle = 0.0;
  float errorDot = 0.0;
  float errorSum = 0.0;
  float timeDiff = 0.0;
  float currentTime = 0.0;
  float lastTime = 0.0;

  const float springConstant = 265.04;
  const float gearRatio = 80.0/18.0*50.0/12.0;
  const float Kv = 230.0;
};
#endif
