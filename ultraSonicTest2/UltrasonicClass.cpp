#include "UltrasonicClass.h"
#include "Arduino.h"

Ultrasonic::Ultrasonic(servotor32 hexy)
{
	this.hexy = hexy;	
}


void Ultrasonic::Read()
{
	// read sensor
	float newReading = this.hexy.ping();
	
	
	// apply filter.
	if (newReading != 0)
	{
		// TODO: implement Kalmann filtering.
		this.sensorVal = newReading;
		
	}
	
}