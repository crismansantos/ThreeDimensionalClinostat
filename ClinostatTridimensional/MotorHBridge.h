// MotorHBridge.h

#ifndef _MOTORHBRIDGE_h
#define _MOTORHBRIDGE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class MotorHBridgeClass
{
 protected:


 public:
	void init();
};

extern MotorHBridgeClass MotorHBridge;

#endif

