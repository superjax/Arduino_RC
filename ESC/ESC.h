#ifndef ESC_h
#define ESC_h

#define ESC_DEBUG
#ifdef ESC_DEBUG
#include <Serial.h>
#endif

#include <inttypes.h>
#include <Servo.h>

class ESC : Servo
{
public:
  ESC(){};
  bool arm();   
};

#endif
