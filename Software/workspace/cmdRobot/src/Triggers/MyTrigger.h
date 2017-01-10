#ifndef MyTrigger_H
#define MyTrigger_H

#include "WPILib.h"

class MyTrigger: public Trigger
{
public:
	MyTrigger();
	bool Get();
};

#endif
