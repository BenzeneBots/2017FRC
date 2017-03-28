//Vision.cpp
//This file has all of the vision processing code for the 2017 robot


#include "WPILib.h"
#include <ntcore.h>
#include "NetworkTables/NetworkTable.h"
#include "Lookup.h"
#include "Vision.h"

using std::shared_ptr;
using namespace std;
using namespace Vision;

//gets and returns distance from network table
double Vision::getDistance(){
	double piDistance = 0.0;
	shared_ptr<NetworkTable> myTable = NetworkTable::GetTable("Vision"); // setup network table from RPi
	myTable->GetNumber("Distance", piDistance); // get distance from network tables
	return piDistance;
	//interpolate speed
}

//gets and returns interpolated speed
double Vision::getSpeed(){
	return 0.0;
	//visionResultVals vision
	//interpolate speed given distance value
}


//gets and returns interpolated position
double Vision::getHoodPosition(){
	return 0.0;
//	double piPosition = 0.0;
}
//interpolate position

double Vision::getAngleError(){
	shared_ptr<NetworkTable> myTable = NetworkTable::GetTable("Vision"); // setup network table from RPi
	return myTable->GetNumber("Center", 0.0); // get angle from network tables
}

//get Angle Error

/*

		double distance = 0.0 ; // distance provided by RPi is float
		int dist = 0; // distance in look up table is integer

		int speed, position; // speed and position variables
		speed = 45;        // default speed
		position = 99;      // default position

		myTable->GetNumber("Distance", distance); // get distance from network tables
		dist = int(distance); // convert to integer

	    results mytest = interp( dist ); // call linear interpolation function
	    */
