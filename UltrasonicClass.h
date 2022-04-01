#ifndef ULTRASONIC
#define ULTRASONIC


#include "servotor32"
#include "Arduino.h"


class Ultrasonic
{
  public:
    Ultrasonic(servotor32 hexy);
	void Read();
	float sensorVal;
  private:
	void Filter();
	
	servotor32 hexy;
}

#endif