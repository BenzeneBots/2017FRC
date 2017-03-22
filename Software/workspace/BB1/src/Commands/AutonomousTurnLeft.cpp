// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "AutonomousTurnLeft.h"
#include "Pigeon.h"

double leftTurnSpeed = -0.2;
double rightTurnSpeed = -0.2;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutonomousTurnLeft::AutonomousTurnLeft(int angle): Command() {
    m_angle = angle;
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void AutonomousTurnLeft::Initialize() {
	Pigeon::ResetYaw(); //resets the gyro to see the starting position as 0.
}

// Called repeatedly when this Command is scheduled to run
void AutonomousTurnLeft::Execute() {
	Robot::drive->TankDrive(-1.0 * leftTurnSpeed, rightTurnSpeed);
}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousTurnLeft::IsFinished() {
	double yaw = Pigeon::GetYaw();
	if(abs(m_angle - yaw) < 2.0){	//is the angle within 2 degrees of the set point?
	return true;
	}
	else {
		return false;
	}
}

// Called once after isFinished returns true
void AutonomousTurnLeft::End() {
Robot::drive->TankDrive(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutonomousTurnLeft::Interrupted() {

}
