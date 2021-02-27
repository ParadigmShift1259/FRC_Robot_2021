/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <wpi/math>

#include <ctre/phoenix/CANifier.h>

using namespace ctre::phoenix;

// Uncomment this to use Mk2 swerve drive instead of Mk3 swerve drive
//#define Mk2

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace Math
{
    constexpr double kTau = 2.0 * wpi::math::pi;
}

namespace DriveConstants
{
    constexpr int kNumSwerveModules = 4;

    /// \name CAN bus IDs
    ///@{
    /// CAN IDs for swerve modules

    constexpr int kCanifierID = 0;
    
    constexpr int kFrontLeftDriveMotorPort    = 1;          
    constexpr int kFrontLeftTurningMotorPort  = 2;

    constexpr int kFrontRightDriveMotorPort   = 3;
    constexpr int kFrontRightTurningMotorPort = 4;

    constexpr int kRearRightDriveMotorPort    = 5;
    constexpr int kRearRightTurningMotorPort  = 6;

    constexpr int kRearLeftDriveMotorPort     = 7;
    constexpr int kRearLeftTurningMotorPort   = 8;
    ///@}

    /// \name Canifier PWM channels
    ///@{
    /// PWM channels for the canifier
    constexpr CANifier::PWMChannel kFrontLeftPWM = CANifier::PWMChannel::PWMChannel0;
    constexpr CANifier::PWMChannel kFrontRightPWM = CANifier::PWMChannel::PWMChannel2;
    constexpr CANifier::PWMChannel kRearRightPWM = CANifier::PWMChannel::PWMChannel1;
    constexpr CANifier::PWMChannel kRearLeftPWM = CANifier::PWMChannel::PWMChannel3;
    ///@}

    /// \name Roborio analog input channels
    ///@{
    /// Roborio analog input channels for Mk2 absolute encoders
    constexpr int kFrontLeftTurningEncoderPort  = 0;
    constexpr int kFrontRightTurningEncoderPort = 1;
    constexpr int kRearRightTurningEncoderPort  = 2;
    constexpr int kRearLeftTurningEncoderPort   = 3;
    ///@}

    /// \name Drive wheel reversal (inverting) flags
    ///@{
    /// To keep the swerve module bevel gear facing inwards we need to reverse the right side
#ifdef Mk2
    constexpr bool kFrontLeftDriveMotorReversed  = false;
    constexpr bool kRearLeftDriveMotorReversed   = false;
    constexpr bool kFrontRightDriveMotorReversed = true;
    constexpr bool kRearRightDriveMotorReversed  = true;
#else
    constexpr bool kFrontLeftDriveMotorReversed  = true;
    constexpr bool kRearLeftDriveMotorReversed   = true;
    constexpr bool kFrontRightDriveMotorReversed = false;
    constexpr bool kRearRightDriveMotorReversed  = false;
#endif
    ///@}

    constexpr bool kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The RobotPy Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.
    //constexpr auto ks = 1_V;
    //constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
    //constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    //constexpr double kPFrontLeftVel = 0.5;
    //constexpr double kPRearLeftVel = 0.5;
    //constexpr double kPFrontRightVel = 0.5;
    //constexpr double kPRearRightVel = 0.5;

    // Process for reentering values: 0 all values out, line up with stick, all gears face inwards
    // Line up based on side, left or right
    // Record values, enter below, then redeploy
    // All gears should face outwards

#ifdef Mk2
    constexpr double kFrontLeftOffset   = 3.142; //6.412;           //3.142;         // 3.14;
    constexpr double kFrontRightOffset  = 5.105; //5.155 + 1.57;    //5.105;         // 5.07;         //5.66;
    constexpr double kRearLeftOffset    = 5.963; //1.6292;  //1.8292; //4.85;       //1.42921;       // 3.34;         //4.29;
    constexpr double kRearRightOffset   = 0.665; //0.635 + 1.57;    //0.665;         // 0.63;         //5.29;
    // constexpr double kFrontLeftOffset   = 3.142;
    // constexpr double kFrontRightOffset  = 5.105 - wpi::math::pi;
    // constexpr double kRearLeftOffset    = 1.8292;
    // constexpr double kRearRightOffset   = 0.665 + wpi::math::pi;
#else
    //Mk3 swerve module
    constexpr double kFrontLeftOffset   = 2718.0; // 2.163;
    constexpr double kFrontRightOffset  =  238.0; // 5.897;
    constexpr double kRearRightOffset   = 1861.0; // 3.405;
    constexpr double kRearLeftOffset    = 37.0; // 0.351;
#endif

    constexpr double kMaxAnalogVoltage = 4.93;                              //!< Absolute encoder runs 0 to 4.93V
    constexpr double kTurnVoltageToRadians = 2.0 * wpi::math::pi / kMaxAnalogVoltage;
    constexpr double KTurnVoltageToDegrees = 360 / kMaxAnalogVoltage;

    constexpr double kPulseWidthToZeroOne = 4096.0;    // 4096 micro second pulse width is full circle
    constexpr double kPulseWidthToRadians =  Math::kTau / kPulseWidthToZeroOne;

    /// Turn PID Controller for Swerve Modules
#ifdef Mk2
    constexpr double kTurnP = 0.35; // 0.35 // 0.1
    constexpr double kTurnI = 0; //1e-4;
    constexpr double kTurnD = 1.85; // 1.85 // 1
#else
    constexpr double kTurnP = 0.1;   //0.35;
    constexpr double kTurnI = 0.0;   //1e-4;
    constexpr double kTurnD = 1.1;   // 1.85
#endif

    /// \name Robot Rotation PID Controller
    ///@{
    /// Rotation PID Controller for Rotation Drive, converts between radians angle error to radians per second turn
    constexpr double kRotationP = 2;
    constexpr double kRotationI = 0;
    constexpr double kRotationIMaxRange = 0.30;
    constexpr double kRotationD = 0.05;
    ///@}

    constexpr double kMaxAbsoluteRotationSpeed = 2.5;
    constexpr double kMaxAbsoluteTurnableSpeed = 1.5;
    constexpr double kAbsoluteRotationTolerance = 0.04;

    constexpr double kMinTurnPrioritySpeed = 0.4;

}  // namespace DriveConstants

namespace ModuleConstants
{

    constexpr int kEncoderCPR = 1024;
    //constexpr int kEncoderCPR = 2048; Mk3
    constexpr int kEncoderTicksPerSec = 10;                 //!< TalonFX::GetSelectedSensorVelocity() returns ticks/100ms = 10 ticks/sec
    constexpr double kWheelDiameterMeters = .1016;          //!< 4"
#ifdef Mk2
    constexpr double kDriveGearRatio = 8.31;              //!< MK2 swerve modules 11.9 ft/sec
    constexpr double kTurnMotorRevsPerWheelRev = 18.0;  // 12.8 Mk3
    constexpr double kDriveEncoderDistancePerPulse = (kWheelDiameterMeters * wpi::math::pi) / static_cast<double>(kEncoderCPR);
#else
    constexpr double kDriveGearRatio = 8.16;                //!< MK3 swerve modules w/NEOs 12.1 ft/sec w/Falcon 13.6 ft/sec
    constexpr double kTurnMotorRevsPerWheelRev = 12.8;
    /// Assumes the encoders are directly mounted on the wheel shafts
    // ticks / 100 ms -> ticks / s -> motor rev / s -> wheel rev / s -> m / s
    constexpr double kDriveEncoderMetersPerSec = kEncoderTicksPerSec / static_cast<double>(kEncoderCPR) / kDriveGearRatio * (kWheelDiameterMeters * wpi::math::pi);
#endif
    constexpr double kTurnEncoderCPR = 4096.0 / kTurnMotorRevsPerWheelRev;    // Mag encoder relative output to SparkMax

#ifdef Mk2
    constexpr double kP_ModuleTurningController = 1.1;
#else
    constexpr double kP_ModuleTurningController = 0.01;
#endif
    constexpr double kD_ModuleTurningController = 0.03;

    constexpr double kPModuleDriveController = 0.001;

    constexpr uint kMotorCurrentLimit = 30;
}   // namespace ModuleConstants

namespace AutoConstants
{
    using radians_per_second_squared_t = units::compound_unit<units::radians, units::inverse<units::squared<units::second>>>;

    constexpr auto kMaxSpeed = units::meters_per_second_t(3.6);                 // 1.0   // 5.0
    constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(1.0);  // 0.1
    constexpr auto kMaxAngularSpeed = units::radians_per_second_t(wpi::math::pi);
    constexpr auto kMaxAngularAcceleration = units::unit_t<radians_per_second_squared_t>(wpi::math::pi);

#ifdef Mk2
    constexpr double kPXController = 2.0;       // 0.25 
    constexpr double kPYController = 2.0;       // 0.25
    constexpr double kPThetaController = 4.0;   // 2.0
#else
    constexpr double kPXController = 0.25;
    constexpr double kPYController = 0.25;
    constexpr double kPThetaController = 0.5;
#endif

    extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants
{
    constexpr double kDeadzoneX = 0.10;
    constexpr double kDeadzoneY = 0.10;
    constexpr double kDeadzoneRot = 0.10;
    constexpr double kDeadzoneAbsRot = 0.50;
    constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants

// Intake Subsystem constants
namespace IntakeConstants
{
    constexpr double kMotorPort = 15;   // 0
    constexpr double kMotorReverseConstant = 1;

    constexpr double kIngestLow = 0.3;
    constexpr double kIngestHigh = 0.6;
    constexpr double kReleaseLow = -0.3;
    constexpr double kReleaseHigh = -0.6;
}

// Flywheel Subsystem constants
namespace FlywheelConstants
{
    constexpr double kMotorPort = 14;

    constexpr double kRampRate = 0.1;
    // Total error allowed for the flywheel, in RPM
    constexpr double kAllowedError = 20;

    constexpr double kP = 0;
    constexpr double kI = 0;
    constexpr double kD = 0;

    constexpr double kMinOut = 0;
    constexpr double kMaxOut = 1.0;

    constexpr double kS = 0;
    constexpr double kV = 0;
    constexpr double kA = 0;

    // Diameter is in meters
    constexpr double kWheelDiameter = 0.1524;
    constexpr double kSecondsPerMinute = 60;
    constexpr double kWheelMetersPerRev = kWheelDiameter * wpi::math::pi;
    // Meters per second to Revolutions per minute
    constexpr double kMPSPerRPM = kWheelMetersPerRev / kSecondsPerMinute;

    /// Use MPSPerRPM to determine the ramp rates, current values are just placeholders
    constexpr double kIdleRPM = 10; 
    constexpr double kRampRPM = 15;
}

// Turret Subsystem Constants
namespace TurretConstants
{
    constexpr double kMotorPort = 12;

    constexpr double kP = 0;
    constexpr double kI = 0;
    constexpr double kD = 0;

    constexpr double kMinOut = 0;
    constexpr double kMaxOut = 0.175;

    constexpr double kTimeout = 30;
    constexpr double kInverted = true;
    constexpr double kSensorPhase = true;

    constexpr double kDegreeStopRange = 0.125;

    constexpr double kPulley = 2.7305;
    constexpr double kSpinner = 29.845;

    // The motor on the turret drives a pulley, while drives the turret
    // MotorRev indicates the revolution of the motor, while Rev indicates the revolution of the turret
    constexpr double kMotorRevPerRev = kPulley / kSpinner;
    constexpr double kTicksPerRev = 4096.0;
    constexpr double kDegreesPerRev = 360.0;
    constexpr double kRadiansPerRev = wpi::math::pi * 2.0;

    // Offset of origin point of turret angle and robot angle, in degrees. Robot 0 is forward
    constexpr double kTurretToRobotAngleOffset = -45;
    // Maximum rotation of the turret relative to the turret, in degrees
    constexpr double kMinAngle = 0;
    constexpr double kMaxAngle = 270;

    constexpr double kAddedAngle = 10;

    // initial configured angle of the turret relative to the turret, in degrees
    constexpr double kStartingPositionDegrees = 135;
}

/// Hood subsystem constants
namespace HoodConstants
{
    /// PWM Port for hood servo
    constexpr int kPWMPort = 13;
    constexpr double kTestServoSpeed = 0.14;

}

// Cycler Subsystem Constants
namespace CyclerConstants
{
    constexpr double kFeederPort = 14;
    constexpr double kTurnTablePort = 15;

    constexpr double kFeederSpeed = 0.400;
    constexpr double kTurnTableSpeed = 0.400;

    // Time to go from 0 to full throttle
    constexpr double kTurnTableRampRate = 0.5;

    constexpr double kTimePassed = 0.20;

    constexpr double kFeederTimeout = 10;
    constexpr double kFeederTimeoutAlt = 15;

    constexpr double kMinOut = 0;
    constexpr double kMaxOut = 0.175;

    constexpr double kTimeout = 30;
    constexpr double kInverted = true;
    constexpr double kSensorPhase = true;

    constexpr double kDegreeStopRange = 0.125;

    constexpr double kPulley = 2.7305;
    constexpr double kSpinner = 29.845;

    constexpr double kMotorRevPerRev = kPulley / kSpinner;
    constexpr double kTicksPerRev = 4096.0;
    constexpr double kDegreesPerRev = 360.0;
    constexpr double kRadiansPerRev = wpi::math::pi * 2.0;

    // Maximum rotation of the turntable relative to the turntable, in degrees
    constexpr double kMinAngle = 0;
    constexpr double kMaxAngle = 270;

    constexpr double kAddedAngle = 10;

    // initial configured angle of the turntable relative to the turntable, in degrees
    constexpr double kStartingPositionDegrees = 135;
}

// Vision Subsystem Constants
namespace VisionConstants
{
    // Mounting angle of the limelight, in degrees
    constexpr double kMountingAngle = 18.0;
    // Mounting height of the limelight from the ground, in inches
    constexpr double kMountingHeight = 21.5;
    // Target center height, in inches
    constexpr double kTargetHeight = 98.25;
    // Target width, in inches
    constexpr double kTargetSize = 15;
}

// Climber Subsystem constants
namespace ClimberConstants
{
    constexpr double kMotorPort = 13;   // 0
    constexpr double kMotorReverseConstant = 1;

}