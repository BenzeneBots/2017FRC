// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.




#include "Shooter.h"
#include "../RobotMap.h"
#include "CANTalon.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

Shooter::Shooter() : Subsystem("Shooter") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    shooterMotor = RobotMap::shooterShooterMotor;
    hood = RobotMap::shooterHood;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}

void Shooter::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}


// Put methods for controlling this subsystem
// here. Call these from Commands.

void Shooter::Spin(){

	// // can talon intiatize and set the number of pulses per revolution , set PID , rpm
	shooterMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	shooterMotor->SetSensorDirection(false);
	shooterMotor->ConfigEncoderCodesPerRev(4096);
	shooterMotor->SetAllowableClosedLoopErr(0);
	shooterMotor->SelectProfileSlot(0);

	shooterMotor->SetF(0.0);
	shooterMotor->SetP(0.5);
	shooterMotor->SetI(0.0);
	shooterMotor->SetD(0.0);

    shooterMotor->SetControlMode(frc::CANSpeedController::kSpeed);
	shooterMotor->Set(120);


	printf("Spinning\n");
}

void Shooter::Stop(){
	shooterMotor->Set(0);

	printf("STOP shooter\n");
}
void Shooter::SetHoodPosition(double position){
	hood->Set(position);
}

