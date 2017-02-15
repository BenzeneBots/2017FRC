#include <memory>

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "Commands/ExampleCommand.h"
#include "CommandBase.h"

#include "WPILib.h"
#include "PigeonImu.h"
#include "RobotMap.h"
#include "Timer.h"
#include "NetworkTables/NetworkTable.h"

PigeonImu * imu;
Servo * lin;
std::shared_ptr<NetworkTable> table;
CANTalon * shooterMotor;

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() override {
		chooser.AddDefault("Default Auto", new ExampleCommand());
		// chooser.AddObject("My Auto", new MyAutoCommand());
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		imu = new PigeonImu( 16 );								// Inertial Measurement Unit
		imu->SetFusedHeading( 0.0 );

		shooterMotor = new CANTalon( 14 );						// Shooter Motor
		shooterMotor->SetExpiration( 0.5 );
		shooterMotor->SetSafetyEnabled( true );

		CameraServer::GetInstance()->StartAutomaticCapture();	// USB on RoboRio
		table = NetworkTable::GetTable( "Vision" );				// Network Table from Raspberry Pi
		lin = new Servo( 9 );									// Linear Actuator Servo

		printf( "Robot Init\n" );
	}

	void RobotPeriodic() override {

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {
		printf( "Disable Init\n" );
	}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		/* std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", "Default");
		if (autoSelected == "My Auto") {
			autonomousCommand.reset(new MyAutoCommand());
		}
		else {
			autonomousCommand.reset(new ExampleCommand());
		} */

		autonomousCommand.reset(chooser.GetSelected());

		if (autonomousCommand.get() != nullptr) {
			autonomousCommand->Start();
		}

		printf( "Auto Init\n" );
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
		}
		printf( "TeleOp Init\n" );
	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TestPeriodic() override {
		char s[100];			// Temp string area.
		bool flgVal = true;		// Valid value flag (assume valid).
		static bool fIMUCal = false;

		frc::LiveWindow::GetInstance()->Run();

		// Note, the 'String' and 'Button' can be read or written to.  However, the
		// 'LED' can only be written to from the RoboRio.

		// FPGA Runtime String Display
		// ------------------------------------------------------------------------
		double tm = Timer::GetFPGATimestamp();			// Runtime in seconds.
		sprintf( s, "Run Time: %ld", (long)tm );
		SmartDashboard::PutString("DB/String 0", s);	// Write to dashboard in string slot zero.

		// Pi Heartbeat Counter String Display
		// ------------------------------------------------------------------------
		double nCont = table->GetNumber( "PiCounter", 0.0 );
		sprintf( s, "Pi Count: %ld", (long)nCont );
		SmartDashboard::PutString("DB/String 5", s);

		// IMU Fused Heading & Temperature Calibration
		// ------------------------------------------------------------------------
		double imuHeading = imu->GetFusedHeading();
		sprintf( s, "IMU Heading: %0.2f", imuHeading );
		SmartDashboard::PutString("DB/String 6", s);

		bool imuCal = SmartDashboard::GetBoolean( "DB/Button 2", false );
		if( imuCal && !fIMUCal ) {
			fIMUCal = true;
			imu->EnterCalibrationMode(imu->Temperature);
			printf( "IMU Temp Cal Started\n" );
		}

		// Linear Actuator Stuff
		// ------------------------------------------------------------------------
		// Read state of button from dashboard.
		bool linEn = SmartDashboard::GetBoolean( "DB/Button 0", false );

		// The slider returns 0.0 to 5.0 values.
		double linCmd = SmartDashboard::GetNumber("DB/Slider 0", 0.0);
		sprintf( s, "Lin: %0.2f", linCmd / 5.0 );
		SmartDashboard::PutString("DB/String 1", s);	// Display scaled value as string 0.0 to 1.0.

		if( linCmd > 4.0 ) {	// Clamp max output to a max of 4.0 (or 0.8 scaled).
			linCmd = 4.0;
			flgVal = false;
		}
		if( linCmd < 1.0 ) {	// Clamp min value to 1.0 (or 0.2 scaled).
			linCmd = 1.0;
			flgVal = false;
		}
		// If slider is in the valid range then turn on the BIG LED.
		SmartDashboard::PutBoolean( "DB/LED 0", flgVal );	// Note, LED can only be written to.

		// If the button is on, set the linear actuator to a value between 0.2 to 0.8.
		if( linEn ) lin->Set( linCmd / 5.0 );
		else 		lin->Set( 1.0 / 5.0 );		// Else, set to min value if button is off.

		// Shooter Motor Control & Test
		// ------------------------------------------------------------------------
		bool dirShooter = SmartDashboard::GetBoolean( "DB/Button 1", true );
		double spShooter = SmartDashboard::GetNumber( "DB/Slider 1", 0.0 ) / 5.0;
		if( dirShooter ) spShooter *= -1.0;
		shooterMotor->Set( spShooter );
		double sp = shooterMotor->GetSpeed();
		sprintf( s, "Speed: %0.2f", sp );
		SmartDashboard::PutString("DB/String 2", s);
	}

	void TestInit() override {
		//PigeonImu::GeneralStatus genStatus;
		//imu->GetGeneralStatus(genStatus);
		//printf( "IMU Temp: %f\n", genStatus.tempC );

		// Setup SmartDashboard controls to default values.
		SmartDashboard::PutNumber( "DB/Slider 0", 0.0 );	// Zero Slider for Linear Act Control.
		SmartDashboard::PutNumber( "DB/Slider 1", 0.0 );	// Init Slider to Zero Shooter Speed.
		SmartDashboard::PutBoolean( "DB/Button 0", false );	// Default to not driving linear actuator.
		SmartDashboard::PutBoolean( "DB/Button 1", true );	// Default to forward.
		SmartDashboard::PutBoolean( "DB/Button 2", false );	// Default to not calibrate IMU Temp.

		printf( "Test Init\n" );
	}

private:
	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> chooser;
};

START_ROBOT_CLASS(Robot)
