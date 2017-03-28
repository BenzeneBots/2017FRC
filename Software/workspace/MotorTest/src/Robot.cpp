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
#include "CANTalon.h"
#include "NetworkTables/NetworkTable.h"
#include "PigeonImu.h"
#include "Joystick.h"

#define HDMAX	0.25	// Max absolute motor rotation allowed on hood angle.
#define HDMIN	0.0	// Min hood rotation.

class Robot: public frc::IterativeRobot {

CANTalon *m1;
CANTalon *hoodMotor;
std::shared_ptr<NetworkTable> table;
PigeonImu *imu;
Victor *leftDrive;
Victor *rightDrive;
Victor *intakeMotor;
Encoder *leftPos;
Encoder *rightPos;
PowerDistributionPanel *pdb;
Joystick *xbox1;
//PIDController *leftPID;

public:
	//std::shared_ptr<SpeedController> m1;

	void RobotInit() override {
		//chooser.AddDefault("Default Auto", new ExampleCommand());
		//chooser.AddObject("My Auto", new MyAutoCommand());
		//frc::SmartDashboard::PutData("Auto Modes", &chooser);

		xbox1 = new Joystick( 0 );

		pdb = new PowerDistributionPanel( 0 );

		imu = new PigeonImu( 14 );

		leftDrive = new Victor( 0 );
		leftDrive->SetSafetyEnabled( false );
		rightDrive = new Victor( 1 );
		rightDrive->SetSafetyEnabled( false );
		intakeMotor = new Victor( 2 );
		intakeMotor->SetSafetyEnabled( false );

		leftPos = new Encoder( 0, 1, Encoder::EncodingType::k4X );
		//leftPos->SetMaxPeriod( 10000.0 );
		//leftPos->SetMinRate( -10000.0 );
		leftPos->Reset();

		rightPos = new Encoder( 2, 3, Encoder::EncodingType::k4X );
		rightPos->Reset();

		/*
		double kP=1.0, kI=0.0, kD=0.0, kFF=0.0;
		leftPID = new PIDController( kP, kI, kD, kFF, leftPos, leftDrive );
		leftPID->SetPIDSourceType( PIDSourceType::kRate );
		leftPID->SetOutputRange( -0.2, 0.2 );
		leftPID->SetInputRange( -1000.0, 1000.0 );
		leftPID->SetSetpoint( 0.0 );
		leftPID->Enable();
		*/

		hoodMotor = new CANTalon( 16 );
		hoodMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		hoodMotor->SetSensorDirection(true);
		hoodMotor->ConfigNominalOutputVoltage( +0., -0. );
		hoodMotor->ConfigPeakOutputVoltage( +12., -12. );	// Use +/- 12 Volts on output.
		hoodMotor->SetAllowableClosedLoopErr( 5 );			// Drive motor is outside these encoder counts.
		hoodMotor->SelectProfileSlot( 0 );					// Use slot zero.
		hoodMotor->SetF( 0.0 );
		hoodMotor->SetP( 1.0 );
		hoodMotor->SetI( 0.001 );
		hoodMotor->SetD( 0.0 );
		//hoodMotor->ConfigSoftPositionLimits( 4096 * 0.25, 4096 * 0.12 );	// Use web page to set these instead.
		hoodMotor->SetIzone( 100 );
		hoodMotor->SetVoltageRampRate( 10.0 );				// 1.2 Volts/Second is the slowest possible ramp rate!

		hoodMotor->SetControlMode(CANSpeedController::kPosition);
		hoodMotor->Set( hoodMotor->GetPosition() );			// Sync set point with current position.

		// Setup the shooter motor.
		m1 = new CANTalon( 15 );
		m1->SetTalonControlMode( CANTalon::kDisabled );

		m1->SelectProfileSlot( 1 );											// Use slot one (can be zero or one).
		//m1->SetExpiration( 0.5 );

		m1->SetFeedbackDevice( CANTalon::CtreMagEncoder_Absolute );
		m1->SetSensorDirection( true );
		m1->ConfigNominalOutputVoltage( +0.f, -0.f );
		m1->ConfigPeakOutputVoltage( +12.f, -12.f );
		m1->ConfigLimitMode( CANTalon::kLimitMode_SrxDisableSwitchInputs );	// Disable Limit SW and Soft Limits.
		m1->SetVoltageRampRate( 12.0 );										// Ramp: (Xsec/V / 12V)
		m1->ConfigEncoderCodesPerRev( 4096 );

		/*
		m1->SetCloseLoopRampRate( 0.0 );
		m1->SetIzone( 0.0 );
		m1->SetAllowableClosedLoopErr( 0 );
		*/

		m1->SelectProfileSlot( 0 );				// Setup Slot Zero with Zero I
		// Sets:     P    I    D     FF
		m1->SetPID( 0.1, 0.0, 0.0, 0.0328 );	// I of PID = 0.0
		m1->SelectProfileSlot( 1 );				// Go back to Slot1 by default.
		m1->SetPID( 0.1, 0.0002, 0.0, 0.0328 );	// Normal PID+FF values.
		m1->ClearIaccum();

		m1->SetTalonControlMode( CANTalon::kDisabled );
		m1->SetSafetyEnabled( false );

		table = NetworkTable::GetTable( "Vision" );

		printf( "Robot Init\n" );
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {
		leftDrive->Set( 0.0 );
		rightDrive->Set( 0.0 );
		intakeMotor->Set( 0.0 );
		printf( "Disabled\n" );
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

		//autonomousCommand.reset(chooser.GetSelected());

		//if (autonomousCommand.get() != nullptr) {
		//	autonomousCommand->Start();
		//}

		printf( "AutoInit\n" );
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		//if (autonomousCommand != nullptr) {
		//	autonomousCommand->Cancel();
		//}
		printf( "TeleopInit\n" );
	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	// When enFlg is true, auto align and center the robot onto the target. Call this
	// function often from a periodic loop.
	// ========================================================================
	int visAutoCenter( bool enFlg, double visDist, double visCenter, bool visLock ) {
		int ret = 0;									// 1=Done, 0=Working, -1=Timeout
		const int cntTO_Reset = 500;					// Timeout Reset Value
		static int printLoops=0, cntTO=cntTO_Reset;
		static double out=0.0;
		//char s[100];

		const double rate = 0.005;
		const double maxOut = 0.25;
		const double deadband = 0.03;

		//sscanf( SmartDashboard::GetString( "DB/String 0", "0.01" ), "%f", &rate );
		//const double maxOut = scanf( "%f", SmartDashboard::GetString( "DB/String 1", "0.0" ) );
		//const double deadband = scanf( "%f", SmartDashboard::GetString( "DB/String 2", "10.0" ) );

		if( enFlg == false ) {
			out = 0.0;
			ret = -1;
		}

		if( enFlg && (visLock == false) ) {
			leftDrive->Set( 0.0 );
			rightDrive->Set( 0.0 );
			out = 0.0;
			ret = -1;
		}

		if( enFlg && visLock ) {
			// Test if we're in the deadband...
			if( (visCenter < deadband) && (visCenter > (deadband*-1.0)) ) {
				printf( "visAutoCenter Complete\n" );
				cntTO = cntTO_Reset;
				ret = 1;
				leftDrive->Set( 0.0 );
				rightDrive->Set( 0.0 );
				out = 0.0;
			}
			else {
				if( --cntTO <= 0 ) {
					leftDrive->Set( 0.0 );
					rightDrive->Set( 0.0 );
					cntTO = cntTO_Reset;
					ret = -1;					// Return a timeout state.
					out = 0.0;
				}
				else {
					ret = 0;
					if( out < maxOut )	out += rate;
					if( visCenter > 0.0 ) {
						leftDrive->Set( out );
						rightDrive->Set( out );
					}
					else {
						leftDrive->Set( out * -1.0 );
						rightDrive->Set( out * -1.0  );
					}
				}
			}
		}
		else {
			cntTO = cntTO_Reset;	// Reset timeout to auto center.
			ret = -1;				// If enable or lock is false then return timeout.
			out = 0.0;
		}

		// Paced debug printing...
		if( --printLoops <= 0 ) {
			printLoops = 30;
			//printf( "visAutoCenter: %0.2f\n", out );
		}

		return ret;
	}

	void TestPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
		static int visCntOld=0, visTimeout=0;
		static int piDebugTO = 50;
		static bool visDebugOld = false;
		static double visDist=0.0, visCenter=0.0;
		static bool visLock=false;

		//static int printLoops = 0;
		static int m1Start = 0;
		//char s[100];
		//static bool fIMUCal = false;

		//frc::LiveWindow::GetInstance()->Run();

		//if( ++printLoops > 50 ) {
		//	printLoops = 0;
		//	printf( "Sh :: %0.2f\n", shSpSetPoint );
		//}

		/*
		//bool imuCal = SmartDashboard::GetBoolean( "DB/Button 2", false );
		//if( imuCal && !fIMUCal ) {
		//	fIMUCal = true;
		//	imu->EnterCalibrationMode(imu->Temperature);
		//	printf( "IMU Temp Cal Started\n" );
		//}

		*/

		// Drive System
		// --------------------------------------------------------------------
		double imuHeading = imu->GetFusedHeading();
		//sprintf( s, "IMU Heading: %0.2f", imuHeading );
		//SmartDashboard::PutString("DB/String 6", s);
		SmartDashboard::PutNumber( "imuHeading", imuHeading );
		//SmartDashboard::PutNumber( "leftEncoder", leftPos->GetRaw() );
		SmartDashboard::PutNumber( "leftEncoder", leftPos->GetRaw() );
		SmartDashboard::PutNumber( "rightEncoder", rightPos->GetRaw() );
		SmartDashboard::PutNumber( "deltaEncoder", leftPos->GetRaw() - rightPos->GetRaw() );

		// If the encoder reset button is pressed, reset it and the two encoders.
		if( SmartDashboard::GetBoolean( "encoderReset", false ) ) {
			leftPos->Reset();
			rightPos->Reset();
			SmartDashboard::PutBoolean( "encoderReset", false );
		}

		// Based on a toggle switch on the dashboard, either use the dashboard or the joystick to
		// control the drive motors.
		if( SmartDashboard::GetBoolean("joyEn", false) == false ) {

			double lMotor = SmartDashboard::GetNumber( "leftDrive", 0.0 );
			leftDrive->Set( lMotor / 100.0 );
			double rMotor = SmartDashboard::GetNumber( "rightDrive", 0.0 );
			rightDrive->Set( rMotor / 100.0 );

		}
		else {
			// This code drives the motors in Arcade style using the joystick input.
			double yJoy = xbox1->GetY() * -0.4;		// Flip Y axis and scale to 40%.
			double xJoy = xbox1->GetX() * 0.4;		// Just scale to the same as Y.
			double z = xbox1->GetZ() * 0.4;			// Left bumper throttle - used to drive strait ahead.

			// If not much bumper throttle then drive arcade style.
			if( z < 0.02 ) {
				leftDrive->Set(  yJoy + xJoy );
				rightDrive->Set( yJoy - xJoy );
			}
			// Else, use the bumper throttle to drive strait ahead.
			else {
				leftDrive->Set(  z );
				rightDrive->Set( z );
			}
		}

		bool imuReset = SmartDashboard::GetBoolean( "imuReset", false );
		if( imuReset ) {
			imu->SetFusedHeading( 0.0 );
			SmartDashboard::PutBoolean( "imuReset", false );
		}

		double m1Amps = pdb->GetCurrent( 0 );	SmartDashboard::PutNumber( "m1Current", abs(m1Amps) );
		double m2Amps = pdb->GetCurrent( 1 );	SmartDashboard::PutNumber( "m2Current", abs(m2Amps) );
		double m3Amps = pdb->GetCurrent( 2 );	SmartDashboard::PutNumber( "m3Current", abs(m3Amps) );
		double m4Amps = pdb->GetCurrent( 3 );	SmartDashboard::PutNumber( "m4Current", abs(m4Amps) );
		double m5Amps = pdb->GetCurrent( 4 );	SmartDashboard::PutNumber( "m5Current", abs(m5Amps) );
		double m6Amps = pdb->GetCurrent( 5 );	SmartDashboard::PutNumber( "m6Current", abs(m6Amps) );

		// Shooter Control Code...
		// --------------------------------------------------------------------
		bool shEnable = SmartDashboard::GetBoolean( "shEnable", false );			// Enable Toggle Switch
		double shSpSetPoint = SmartDashboard::GetNumber( "shSpSetPoint", 0.0 );		// Set Point Dial
		static bool flg = false;

		// Run the motor if the button is on.
		if( shEnable ) {
			if( flg == false ) {
				flg = true;
				printf( "Enabled\n" );
				m1->SetTalonControlMode( CANTalon::kSpeedMode );
			}
			// For the first few milliseconds, start with no integral in the PID.
			if( ++m1Start < 50 )	m1->SelectProfileSlot( 0 );	// Slot Zero has integral set to zero.
			else					m1->SelectProfileSlot( 1 );	// Slot One is tuned for steady state.
		}
		// Else, turn off the motor by disabling it - coast to stop.
		else {
			if( flg == true ) {
				printf( "Disabled\n" );
				m1->SetTalonControlMode( CANTalon::kDisabled );
			}
			flg = false;
			shSpSetPoint = 0.0;
			m1Start = 0;			// Reset startup counter.
		}
		m1->Set( shSpSetPoint );

		SmartDashboard::PutNumber( "shSpeed", m1->GetSpeed() );
		SmartDashboard::PutNumber( "shOut", m1->GetOutputVoltage() / 12.0 * 100.0 );
		SmartDashboard::PutNumber( "shErr", m1->GetClosedLoopError() );
		SmartDashboard::PutNumber( "shCurrent", m1->GetOutputCurrent() );

		// Climber Control Code...
		// --------------------------------------------------------------------


		// Vision / Network Table IO
		// --------------------------------------------------------------------
		int visCnt = int( table->GetNumber( "Counter", 0.0 ) );		// Get rolling Pi counter.
		// If Pi counter is NOT moving then decreament a timeout counter towards zero.
		if( visCnt == visCntOld ) {
			if( --visTimeout <= 0 ) {	// On timeout hitting zero, clear everything.
				visTimeout = 0;
				SmartDashboard::PutBoolean( "visOK", false );
				SmartDashboard::PutBoolean( "visLock", false );
				SmartDashboard::PutNumber( "visCenter", 0.0 );
				SmartDashboard::PutNumber( "visDist", 0.0 );
			}
		}
		// Else, the Pi counter IS moving so all is good.
		else {
			visCenter = table->GetNumber( "Center", 0.0 );
			SmartDashboard::PutNumber( "visCenter", visCenter );
			visDist = table->GetNumber( "Distance", 0.0 );
			SmartDashboard::PutNumber( "visDist", visDist );
			visLock = table->GetBoolean( "TargetFound", false );
			if( visLock  )  SmartDashboard::PutBoolean( "visLock", true );
			else			SmartDashboard::PutBoolean( "visLock", false );
			SmartDashboard::PutBoolean( "visOK", true );

			visCntOld = visCnt;		// Update old compare counter.
			visTimeout = 50;		// Reset the timeout value.
		}

		bool visDebug = SmartDashboard::GetBoolean( "visDebug", false );
		table->PutBoolean( "Debug", visDebug );
		if( visDebug != visDebugOld ) {
			piDebugTO = 100;
			visDebugOld = visDebug;
		}
		if( piDebugTO > 0 ) {
			piDebugTO -= 1;
			if( piDebugTO == 0 ) SmartDashboard::PutBoolean( "visDebug", false );
		}

		bool visAutoCenterFlg = SmartDashboard::GetBoolean( "visAutoCenter", false );
		int flgVisAuto = visAutoCenter( visAutoCenterFlg, visDist, visCenter, visLock );
		if( (flgVisAuto == -1) || (flgVisAuto == 1) ) {
			SmartDashboard::PutBoolean( "visAutoCenter", false );
		}

		// Shooter Hood Control
		// --------------------------------------------------------------------
		double hdSetPt = SmartDashboard::GetNumber( "hdSetPt", 0.0 );
		// Absolutly make sure the setpoint stays inside the bounds!
		hdSetPt = hdSetPt >= HDMAX ? HDMAX : hdSetPt;		// Clamp variable to a maximum.
		hdSetPt = hdSetPt <=  HDMIN ?  HDMIN : hdSetPt;	// Clamp variable to a minimum.
		hoodMotor->Set( hdSetPt );
		double hdRot = hoodMotor->GetPosition();
		SmartDashboard::PutNumber( "hdPos", hdRot );
	}

	// ========================================================================
	void TestInit() override {

		// Setup stuff to make it safe to enter Test Mode!!!

		// Always init the slider to zero.
		SmartDashboard::PutNumber("DB/Slider 0", 0.0);

		// Init Button Zero off.
		SmartDashboard::PutBoolean( "DB/Button 0", false );

		// Start with shooter disabled and zeroed.
		m1->SetTalonControlMode( CANTalon::kDisabled );		// Shooter PID Controller
		SmartDashboard::PutBoolean( "shEnable", false );	// Shooter Enable Button
		SmartDashboard::PutNumber( "shSpSetPoint", 0.0 );	// Default SetPoint to Zero.

		// Drive System Stuff.
		SmartDashboard::PutNumber( "leftDrive", 0.0 );
		SmartDashboard::PutNumber( "rightDrive", 0.0 );
		SmartDashboard::PutBoolean( "joyEn", false );

		// Shooter Hood Actuator
		SmartDashboard::PutNumber( "hdSetPt", hoodMotor->GetPosition() );


		printf("Test Init\n" );
	}

//private:
//	std::unique_ptr<frc::Command> autonomousCommand;
//	frc::SendableChooser<frc::Command*> chooser;
};

START_ROBOT_CLASS(Robot)
