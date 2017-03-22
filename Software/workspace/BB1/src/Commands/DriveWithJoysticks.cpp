// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "DriveWithJoysticks.h"

#include "Pigeon.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

DriveWithJoysticks::DriveWithJoysticks(double X, double Y, double Z): Command() {
    m_X = X;
    m_Y = Y;
    m_Z = Z;
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::drive.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void DriveWithJoysticks::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoysticks::Execute() {
	Robot::drive->ArcadeDriveJoystick(Robot::oi->getDriver()->GetRawAxis(1),
			Robot::oi->getDriver()->GetRawAxis(4));
	//Murali: Gets joystick values and passes them to the DriveWithJoysticks submethod

	double rightEncVal = Robot::drive->GetRightEncoderValue();
	printf("Right enc: %f\n", rightEncVal);

	double leftEncVal = Robot::drive->GetLeftEncoderValue();
	printf("Left enc: %f\n", leftEncVal);

	double gyro = Pigeon::GetYaw();
	printf("Yaw: %f\n", gyro);

}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoysticks::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void DriveWithJoysticks::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoysticks::Interrupted() {

}
