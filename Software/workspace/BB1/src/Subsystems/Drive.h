// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef DRIVE_H
#define DRIVE_H
#include "Commands/Subsystem.h"
#include "WPILib.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class Drive: public Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<SpeedController> left;
	std::shared_ptr<SpeedController> right;
	std::shared_ptr<RobotDrive> drivetrain;
	std::shared_ptr<Encoder> leftDriveEncoder;
	std::shared_ptr<Encoder> rightDriveEncoder;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	Drive();
	void InitDefaultCommand();
	void ArcadeDriveJoystick(double, double);
	void TankDrive(double, double);
	void stop();
	double GetLeftEncoderValue();
	double GetRightEncoderValue();
	void ResetEncoders();
	double GetLeftRate();
	double GetRightRate();
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
};

#endif
