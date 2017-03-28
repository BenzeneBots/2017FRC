// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "Robot.h"
#include "AutonomousLeftGear.h"

#include "Commands/AutonomousDriveStraight.h"
#include "Commands/AutonomousTurnRight.h"
#include "Commands/AutonomousTurnLeft.h"
#include "Commands/Wait.h"
#include "Commands/GearLoad.h"
#include "Commands/AutonomousShootBalls.h"

#include "Pigeon.h"



// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutonomousLeftGear::AutonomousLeftGear() {
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
     // Add Commands here:
    // e.g. AddSequential(new Command1());
    //      AddSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use AddParallel()
    // e.g. AddParallel(new Command1());
    //      AddSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

	Pigeon::ResetYaw();
	AddSequential(new AutonomousDriveStraight(63));
	AddSequential(new AutonomousTurnLeft(60));
	AddSequential(new AutonomousDriveStraight(60));
	AddSequential(new GearLoad());
	AddSequential(new WaitCommand(3.0));

	// Placed gear, now try to shoot balls
    AddSequential(new AutonomousDriveStraight(-84));
	AddSequential(new AutonomousTurnLeft(15));
	AddSequential(new AutonomousShootBalls(2300, 40));

	//


              // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS
                      // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS
 }