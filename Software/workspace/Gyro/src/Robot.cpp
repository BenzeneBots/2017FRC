#include "WPILib.h"

/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a robot drive
 * straight. This program uses a joystick to drive forwards and backwards while the gyro
 * is used for direction keeping.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 * 
 * 
 */
class Robot: public SampleRobot {
	const int gyroChannel = 0; 			// Analog Input Channel
	const int joystickChannel = 0; 		// USB number in DriverStation

	// Channels for motors
	const int leftMotorChannel = 1;
	const int rightMotorChannel = 0;
	const int leftRearMotorChannel = 3;
	const int rightRearMotorChannel = 2;

	double angleSetpoint = 0.0;
	const double pGain = .005; 		// Proportional turning constant

	// Gyro calibration constant, may need to be adjusted.
	// A gyro value of 360 is set to correspond to one full revolution.
	const double voltsPerDegreePerSecond = .0128;

	RobotDrive myRobot;
	AnalogGyro gyro;
	Joystick joystick;

public:
	Robot() :
	// Create the drivetrain from 4 CAN Talon SRXs.
	myRobot( new Jaguar(leftMotorChannel), new Jaguar(leftRearMotorChannel),
			new Jaguar(rightMotorChannel), new Jaguar(rightRearMotorChannel)),
		// Assign the gyro and joystick channels.
		gyro(gyroChannel), joystick(joystickChannel ) {}

	/**
	 * Runs during autonomous.
	 */
	void Autonomous()
	{

	}

	/**
	 * Sets the gyro sensitivity and drives the robot when the joystick is pushed. The
	 * motor speed is set from the joystick while the RobotDrive turning value is
	 * assigned from the error between the setpoint and the gyro angle.
	 */
	void OperatorControl()
	{
		double turningValue;
		gyro.SetSensitivity(voltsPerDegreePerSecond); //calibrates gyro values to equal degrees

		while ( IsOperatorControl() && IsEnabled() ) {
			turningValue = (angleSetpoint - gyro.GetAngle()) * pGain;

			SmartDashboard::PutNumber("Gyro Rate", turningValue);

			if ( joystick.GetY() <= 0 ) {
				//forwards
				myRobot.Drive(joystick.GetY(), turningValue);
			}
			else {
				//backwards
				myRobot.Drive(joystick.GetY(), -turningValue);
			}
		}
	}

	/**
	 * Runs during test mode.
	 */
	void Test()
	{
		gyro.Reset();

	}
};

START_ROBOT_CLASS(Robot)
