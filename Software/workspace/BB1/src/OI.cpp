// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "OI.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "SmartDashboard/SmartDashboard.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/DriveWithJoysticks.h"
#include "Commands/FloorLoad.h"
#include "Commands/Liftoff.h"
#include "Commands/Shoot.h"


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
    // Process operator interface input here.
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    manipulator.reset(new Joystick(1));
    
    manipA.reset(new JoystickButton(manipulator.get(), 1));
    manipA->WhenPressed(new Shoot());
    manipLT.reset(new JoystickButton(manipulator.get(), 5));
    manipLT->WhenPressed(new FloorLoad());
    driver.reset(new Joystick(0));
    
    drvA.reset(new JoystickButton(driver.get(), 1));
    drvA->WhenPressed(new Liftoff());

    // SmartDashboard Buttons
    SmartDashboard::PutData("Shoot", new Shoot());
    SmartDashboard::PutData("Liftoff", new Liftoff());
    SmartDashboard::PutData("Floor Load", new FloorLoad());
    SmartDashboard::PutData("Drive With Joysticks: defJoy", new DriveWithJoysticks(0, 0, 0));
    SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<Joystick> OI::getDriver() {
   return driver;
}

std::shared_ptr<Joystick> OI::getManipulator() {
   return manipulator;
}


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
