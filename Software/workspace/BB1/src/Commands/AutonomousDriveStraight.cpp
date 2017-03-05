// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "AutonomousDriveStraight.h"

double leftSpeed = .3;
double rightSpeed = .3;	//global variables for the speed of the drive motors

double speedAdjRate =.01;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutonomousDriveStraight::AutonomousDriveStraight(double distance): Command() {
    m_distance = distance;
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drive.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void AutonomousDriveStraight::Initialize() {
	Robot::drive->ResetEncoders(); //Resets encoders so that they start at 0


}

// Called repeatedly when this Command is scheduled to run
void AutonomousDriveStraight::Execute() {

	double leftEncVal = Robot::drive->GetLeftEncoderValue();
	double rightEncVal = Robot::drive->GetRightEncoderValue();	//Gets left and right encoder values in inches

//poll encoder values, if one's ahead of the other, reduce slightly
	if(abs(leftEncVal - rightEncVal) > .5){
		if(leftEncVal > rightEncVal){ //if left side is going faster than right side
			leftSpeed -= speedAdjRate;
		}
		if(leftEncVal < rightEncVal){ //if right side is going faster than left side
			rightSpeed -= speedAdjRate;
		}
	}

	Robot::drive->TankDrive(leftSpeed, rightSpeed);

}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousDriveStraight::IsFinished() {
	double driveEncVal = Robot::drive->GetLeftEncoderValue();
	if(m_distance-driveEncVal < .5){
		return true;
	}
	else{
		return false;
	}
}

// Called once after isFinished returns true
void AutonomousDriveStraight::End() {
	Robot::drive->TankDrive(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutonomousDriveStraight::Interrupted() {

}
