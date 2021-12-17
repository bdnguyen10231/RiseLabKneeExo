#include "fsm.h"
#include "math.h"
#include <arduino.h>

void fsm::setPoint(float rm45, float rtoe, float rheel, float rm12) {
  //float weight = 57.5*98; // newtons
  int x0 = 40;
  float s = 0.5;
  
  float rm45_large = f_large(rm45, s, x0);
  float rtoe_large = f_large(rtoe, s, x0);
  float rheel_large = f_large(rheel, s, x0);
  float rm12_large = f_large(rm12, s, x0);
  
  float rm45_small = 1 - rm45_large;
  float rtoe_small = 1 - rtoe_large;
  float rheel_small = 1 - rheel_large;
  float rm12_small = 1 - rm12_large;

  float IC = rheel_large*rm45_small*rm12_small*rtoe_small;
  float LR = rheel_large*rm45_large*rm12_small*rtoe_small;
  float MS = rheel_large*rm45_large*rm12_large;
  float TS = rheel_small*rm45_large*rm12_large;
  float PS = rheel_small*rm45_small*rm12_small*rtoe_large;
  float SW = rheel_small*rm45_small*rm12_small*rtoe_small;

  if (IC + LR >= 0.8) {spoint = 1;}
  else if (MS >= 0.8) {spoint = 2;}
  else if (TS + PS >= 0.8) {spoint = 3;}
  else if (SW >= 0.8) {spoint = 4;}
  else {spoint = 5;}
}

void fsm::readShoe(float rm45, float rtoe, float rheel, float rm12) {
  
  groundReaction = rtoe+rheel+rm45+rm12;
  setPoint(rm45, rtoe, rheel, rm12);

  if (spoint == 5) {
    spoint = spoint_old;
  } else {spoint_old = spoint;}

  if (spoint == 1) {
    k = 0.01*70;
    angle = 14;
  }else if (spoint == 2) {
    k = 0.01*402;
    angle = 12;
  }else {
    k = 0;
    angle = 0;
  }

  current_time = millis();
  timeDiff = current_time - timeLast;
  timeLast = current_time;
}

float fsm::kong_tanh(float x) {
  float num = exp(2*x) - 1;
  float den = exp(2*x) + 1;
  float idk_tanh = num/den;
  return idk_tanh;
}

float fsm::f_large(float x, float s, int x0) {
  float flarge = 0.5*(fsm::kong_tanh(s*(x-x0))+1);
  return flarge;
}
