//
//  AutoDrive.cpp
//

#include "WPILib.h"
#include "PigeonImu.h"
#include "CANTalon.h"

// Object for IMU

PigeonImu * _pidgey;
_pidgey = new PigeonImu(11); /* Pigeon is on CANBus and has a device ID of 11 */

// Object for Encoder

Encoder *sampleEncoder;
sampleEncoder= new Encoder(0, 1, false, Encoder::EncodingType::k4X);

// Some gains for heading servo,

double kPgain = 0.04; /* percent throttle per degree of error */
double kDgain = 0.0004; /* percent throttle per angular velocity dps */

_pidgey->SetFusedHeading(0.0); /* reset heading, angle measurement wraps at plus/minus 23,040 degrees (64 rotations) */

//Min Rate - Sets the minimum rate before the device is considered stopped. This compensates for both scale factor and distance per pulse and therefore should be entered in engineering units (RPM, RPS, Degrees/sec, In/s, etc)
//Distance Per Pulse - Sets the scale factor between pulses and distance. The library already accounts for the decoding scale factor (1x, 2x, 4x) separately so this value should be set exclusively based on the encoder's Pulses per Revolution and any gearing following the encoder.
//Reverse Direction - Sets the direction the encoder counts, used to flip the direction if the encoder mounting makes the default counting direction unintuitive.
//Samples to Average - Sets the number of samples to average when determining the period. Averaging may be desired to account for mechanical imperfections (such as unevenly spaced reflectors when using a reflective sensor as an encoder) or as oversampling to increase resolution. Valid values are 1 to 127 samples.

sampleEncoder->SetMinRate(10);
sampleEncoder->SetDistancePerPulse(12);   // Gearing ration of 3 x 4 pulses per inch
sampleEncoder->SetReverseDirection(true);
sampleEncoder->SetSamplesToAverage(7);

// The encoder will begin counting as soon as it is created. To reset the encoder value to 0 call reset().

sampleEncoder->Reset();

// Function to turn based on IMU heading

double TurnThrottle (double target_angle) {

        PigeonImu::GeneralStatus genStatus;
        double xyz_dps[3];
        double turnValue;
    
        _pidgey->GetRawGyro(xyz_dps);

        double currentAngle = _pidgey->GetFusedHeading(); // Get fused heading from IMU
        double currentAngularRate = xyz_dps[2];
 
        /* very simple Proportional and Derivative (PD) */
        turnValue = (targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
    
        return turnValue;
};

// Function to return distance from Encoder

double DistanceTravel () {
    return sampleEncoder->GetDistance();
};
