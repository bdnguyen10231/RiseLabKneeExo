#include "hlc.h"
#include "math.h"

#include <arduino.h>

float hlc::controller(float ENC, float vENC, float IMU, float vIMU, float k, int angle, int spoint){
  kneeAngle = ENC;
  kneeAngle_Vel = vENC;
  hipAngle = IMU;
  hipAngle = vIMU;

  if (sinusoidal == 1) {
    reference = amp*sin(freq*current_time*2*M_PI/1000);
  }

  if (manual == 1) {
    reference = manRef;
  }

  // gravity compensation logic
  if (gravity == 1){
    if (spoint == 4){
      if (kneeAngle_Vel > -1){
        reference = -sin((hipAngle + kneeAngle)*M_PI/180)*(0.318*9.8*0.22+1*gravity_Gain);
        }
      else {reference = 0;}
    } 
    else if (impedance == 1){
      reference = k*(angle + kneeAngle)*0.15*impedance_Gain; 
      if (reference < 0){
        reference = 0;
      }
    } 
    else {reference = 0;}
  }

  // safety...
  if (reference > maxTorque) {reference = maxTorque;}
  else if(reference < -maxTorque) {reference = -maxTorque;}

  current_time = millis();
  
  return reference; 
}
