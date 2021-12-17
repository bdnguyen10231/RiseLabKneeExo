#include "llc.h"
#include "math.h"
#define DEBUG false

void llc::controller(float reference, float rawAngle, float kneeAngle) {
  currentTime = millis()/1000.0;
  timeDiff  = currentTime - lastTime;
  lastTime = currentTime;

  measurement = rawAngle*(2*M_PI)/40000*springConstant;

  measurement = measurement + kneeAngle * kneeAngleComp; // eccentricity compensation

  error[2] = error[1];
  error[1] = error[0];
  error[0] = reference - measurement;
  errorDot = (error[0] - error[1])/timeDiff;
  errorDot_F = lowPass_onePole(errorDot, errorDot_F, 0.1);
  
  //signalx = error*Kp + errorDot_F*Kd + errorSum; //+ reference*Kf;
  b[0] = Kp*(1+N*timeDiff) + Ki*timeDiff*(1+N*timeDiff) + Kd*N;
  b[1] = -(Kp*(2+N*timeDiff) + Ki*timeDiff + 2*Kd*N);
  b[2] = Kp + Kd*N;
  a[0] = 1+N*timeDiff;
  a[1] = -(2+N*timeDiff);
  a[2] = 1;
  signalx[2] = signalx[1];
  signalx[1] = signalx[0];
  signalx[0] = -a[1]/a[0]*signalx[1] - a[2]/a[0]*signalx[2] + b[0]/a[0]*error[0] + b[1]/a[0]*error[1] + b[2]/a[0]*error[2];
  
  if (paused == 0){
    //errorSum = errorSum + Ki*error[0]*timeDiff;
    // bounds of signal +- 5nm
    if (signalx[0] > bound) {signalx[0] = bound;}
    else if (signalx[0] < -bound) {signalx[0] = -bound;}
    current = motorDirection*signalx[0]/(8.3/Kv*gearRatio) + feedForwardCurrent;
  }
  else{
    current = 0.0;
  }
}

long llc::lowPass_onePole(float value, float valueLast, float alpha) {
  return alpha*value+(1-alpha)*valueLast;
}
