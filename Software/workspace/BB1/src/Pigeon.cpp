//
//  Pigeon.cpp
//

#include "WPILib.h"
#include "PigeonImu.h"
#include "CANTalon.h"

PigeonImu * pidgey = new PigeonImu(14);

int NormalizeAngle(int angle){
	while (angle < 0){
		angle += 360;
	}
	while(angle > 359){
		angle -= 360;
	}
	return angle;
}

void ResetYaw(){
	pidgey->SetYaw(0.0);
}

double GetYaw(){
	double ypr_array[3];
	pidgey->GetYawPitchRoll(ypr_array);
	return ypr_array[0];
}

double GetFusedHeading(){
	return pidgey->GetFusedHeading();
}
