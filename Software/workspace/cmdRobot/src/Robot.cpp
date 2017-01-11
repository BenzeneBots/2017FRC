#include "WPILib.h"
#include "Commands/Command.h"
#include "Commands/ExampleCommand.h"
#include "CommandBase.h"
#include "CANTalon.h"
//#include "PigeonImu.h"

class Robot: public IterativeRobot
{

private:
	std::unique_ptr<Command> autonomousCommand;
	SendableChooser *chooser;
	CANTalon *m1Motor, *m2Motor;
	PowerDistributionPanel *pdp;
	//PigeonImu *imu;

	void RobotInit()
	{
		CommandBase::init();
		m1Motor = new CANTalon(1);
		m2Motor = new CANTalon(2);
		pdp = new PowerDistributionPanel();
		//imu = new PigeonImu(0);

		chooser = new SendableChooser();
		chooser->AddDefault("Default Auto", new ExampleCommand());
		//chooser->AddObject("My Auto", new MyAutoCommand());
		SmartDashboard::PutData("Auto Modes", chooser);
		//float v = m1Motor->GetBusVoltage();
		float t = pdp->GetTemperature();
		printf( "\n\nBenzene Bots Robot Init...\n\n" );
		printf( "Current PDP Temp: %f\n", t );
		pdp->StartLiveWindowMode();
	}

	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any
     * subsystem information you want to clear when
	 * the robot is disabled.
     */
	void DisabledInit()
	{
		m1Motor->Set( 0.0 );
		m2Motor->Set( 0.0 );
		printf( "Disabled...\n" );
	}

	void DisabledPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit()
	{
		/* std::string autoSelected = SmartDashboard::GetString("Auto Selector", "Default");
		if(autoSelected == "My Auto") {
			autonomousCommand.reset(new MyAutoCommand());
		} else {
			autonomousCommand.reset(new ExampleCommand());
		} */

		autonomousCommand.reset((Command *)chooser->GetSelected());

		if (autonomousCommand != NULL) {
			autonomousCommand->Start();
			printf( "Autonomous Mode...\n" );
		}
	}

	void AutonomousPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}

	void TeleopInit()
	{
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != NULL) {
			autonomousCommand->Cancel();
			printf( "Teleop Mode...\n" );
		}
	}

	void TeleopPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}

	void TestInit() {
		printf( "Test Mode...\n" );
	}

	void TestPeriodic()
	{
		LiveWindow::GetInstance()->Run();
		SmartDashboard::PutNumber("PDP Total Current", pdp->GetTotalCurrent() );
		m1Motor->Set( 0.15 );
		m2Motor->Set( 0.10 );
		//Wait(kUpdatePeriod); // Wait 5ms for the next update.
	}

	void PracticeInit() {
		printf( "Practice Mode...\n" );
	}

};

START_ROBOT_CLASS(Robot)
