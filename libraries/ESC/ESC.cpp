#include "ESC.h"

ESC::arm()
{
  if(attached())
  {

#ifdef ESC_DEBUG
    Serial.print("DEBUG:  Arming ESC attached on pin ");
    Serial.print(servos[this->servoIndex].Pin.nbr,DEC);
#endif
    
    writeMicroseconds(min)
    delay(500);
    writeMicroseconds(max)
    delay(500);
    return 1;
  }
  else
    return 0;
}