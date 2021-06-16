#ifndef fsm_h
#define fsm_h

#include "fsm.h"

class fsm {
  public:
  float k = 0.0;
  int angle = 0;
  int spoint = 0;
  void setPoint(float rm45, float rtoe, float rheel, float rm12);
  void readShoe(float rm45, float rtoe, float rheel, float rm12);

  float groundReaction = 0.0;

  private:
  float current_time = 0.0;
  float timeLast = 0.0;
  float timeDiff = 0.0;
  int spoint_old;
  
  float kong_tanh(float x);
  float f_large(float x, float s, int x0);
};

#endif
