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
#define DualJoysticks

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

    constexpr int kCanifierID = 0;                       //!< CANifier CAN ID (for absolute encoder PWM inputs)
    
    constexpr int kFrontLeftDriveMotorPort    = 1;       //!< Front Left Drive CAN ID (TalonFX)   
    constexpr int kFrontLeftTurningMotorPort  = 2;       //!< Front Left Turn CAN ID (SparkMAX)   

    constexpr int kFrontRightDriveMotorPort   = 3;       //!< Front Right Drive CAN ID (TalonFX)   
    constexpr int kFrontRightTurningMotorPort = 4;       //!< Front Right Turn CAN ID (SparkMAX)

    constexpr int kRearRightDriveMotorPort    = 5;       //!< Rear Right Drive CAN ID (TalonFX)   
    constexpr int kRearRightTurningMotorPort  = 6;       //!< Rear Right Turn CAN ID (SparkMAX)

    constexpr int kRearLeftDriveMotorPort     = 7;       //!< Rear Left Drive CAN ID (TalonFX)   
    constexpr int kRearLeftTurningMotorPort   = 8;       //!< Rear Left Turn CAN ID (SparkMAX)
    ///@}

    /// \name Teleop Drive Constraints
    constexpr auto kDriveSpeed = units::meters_per_second_t(3.0);
    constexpr auto kDriveAngularSpeed = units::radians_per_second_t(wpi::math::pi);

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

    constexpr double kLeftMultipler = 1.21951; // 1.149425;
    ///@}

    constexpr bool kGyroReversed = false;

    // Process for reentering values: 0 all values out, line up with stick, all gears face inwards
    // Line up based on side, left or right
    // Record values, enter below, then redeploy
    // All gears should face outwards

    #ifdef Mk2
    constexpr double kFrontLeftOffset   = (6.28 - 3.14); //3.142; //6.412;           //3.142;         // 3.14;
    constexpr double kFrontRightOffset  = (6.28 - 1.21); //5.105; //5.155 + 1.57;    //5.105;         // 5.07;         //5.66;
    constexpr double kRearLeftOffset    = (6.28 - 0.36); //5.963; //1.6292;  //1.8292; //4.85;       //1.42921;       // 3.34;         //4.29;
    constexpr double kRearRightOffset   = (6.28 - 5.67); //0.665; //0.635 + 1.57;    //0.665;         // 0.63;         //5.29;
    #else
    //Mk3 swerve module
    //============================================LEAVE THESE ZEROES COMMENTED OUT!!!
    // constexpr double kFrontLeftOffset   = 0.0;
    // constexpr double kFrontRightOffset  = 0.0;
    // constexpr double kRearRightOffset   = 0.0;
    // constexpr double kRearLeftOffset    = 0.0;
    //===============================================================================
    constexpr double kFrontLeftOffset   = 2685.0; //2695.0;
    constexpr double kFrontRightOffset  = 198.0; //195.0;
    constexpr double kRearRightOffset   = 1847.0; //1861.0; //1829.0;
    constexpr double kRearLeftOffset    = 2700.0; //2717.0; //486.0; //234.0; //362.891; //147.0;
    #endif

    constexpr double kMaxAnalogVoltage = 4.93;                              //!< Absolute encoder runs 0 to 4.93V
    constexpr double kTurnVoltageToRadians = 2.0 * wpi::math::pi / kMaxAnalogVoltage;
    constexpr double KTurnVoltageToDegrees = 360 / kMaxAnalogVoltage;

    constexpr double kPulseWidthToZeroOne = 4096.0;    // 4096 micro second pulse width is full circle
    constexpr double kPulseWidthToRadians =  Math::kTau / kPulseWidthToZeroOne;

    /// \name Turn PID Controller for Swerve Modules
    ///@{
#ifdef Mk2
    constexpr double kTurnP = 0.35; // 0.35 // 0.1
    constexpr double kTurnI = 0; //1e-4;
    constexpr double kTurnD = 1.85; // 1.85 // 1
#else
    constexpr double kTurnP = 0.35;   //0.35;
    constexpr double kTurnI = 0.0;   //1e-4;
    constexpr double kTurnD = 1.1;   // 1.85
#endif
    ///@}

    /// \name Turn PID Controller for Swerve Modules
    ///@{
        constexpr double kDriveP = 0.1;
        constexpr double kDriveI = 0.0000015;
        constexpr double kDriveD = 0;
    ///@}


    /// \name Robot Rotation PID Controller
    ///@{
    /// Rotation PID Controller for Rotation Drive, converts between radians angle error to radians per second turn
    constexpr double kRotationP = 2;
    constexpr double kRotationI = 0;
    constexpr double kRotationIMaxRange = 0;
    constexpr double kRotationD = 0.085;
    /// Rotation PID Controller additional parameters
    /// Max speed for control
    constexpr double kMaxAbsoluteRotationSpeed = 3.5;
    /// Speeds higher than value will prevent robot from changing directions for a turn
    constexpr double kMaxAbsoluteTurnableSpeed = 3;
    /// Maximum tolerance for turning
    constexpr double kAbsoluteRotationTolerance = 0.07;
    ///@}

    constexpr double kMinTurnPrioritySpeed = 0.4;

}  // namespace DriveConstants

namespace ModuleConstants
{
    #ifdef Mk2
    constexpr int kEncoderCPR = 1024;
    #else
    constexpr int kEncoderCPR = 2048;
    #endif

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
    constexpr double kP_ModuleTurningController = 1.1;
    #endif

    constexpr double kD_ModuleTurningController = 0.03;
    constexpr double kPModuleDriveController = 0.001;

    constexpr uint kMotorCurrentLimit = 30;
}   // namespace ModuleConstants

namespace AutoConstants
{
    using radians_per_second_squared_t = units::compound_unit<units::radians, units::inverse<units::squared<units::second>>>;

    constexpr auto kMaxSpeed = units::meters_per_second_t(3.9);                 // 1.0   // 5.0
    constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(3.0);  // 0.1
    constexpr auto kMaxAngularSpeed = units::radians_per_second_t(wpi::math::pi * 4.0);
    constexpr auto kMaxAngularAcceleration = units::unit_t<radians_per_second_squared_t>(wpi::math::pi * 4.0);

#ifdef Mk2
    constexpr double kPXController = 2.0;       // 0.25 
    constexpr double kPYController = 2.0;       // 0.25
    constexpr double kPThetaController = 4.0;   // 2.0
#else
    constexpr double kPXController = 7.0;
    constexpr double kDXController = 0.7;
    constexpr double kPYController = 7.0;
    constexpr double kDYController = 0.7;
    constexpr double kPThetaController = 6.0;
    constexpr double kDThetaController = 0.9;
#endif

    extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants
{
    constexpr double kDeadzoneX = 0.10;
    constexpr double kDeadzoneY = 0.10;
    constexpr double kDeadzoneRot = 0.10;
    constexpr double kDeadzoneAbsRot = 0.50;
    constexpr int kPrimaryControllerPort = 0;
#ifdef DualJoysticks
    constexpr int kSecondaryControllerPort = 1;
#else
    constexpr int kSecondaryControllerPort = 0;
#endif
}  // namespace OIConstants

// Intake Subsystem constants
namespace IntakeConstants
{
    constexpr double kMotorPort = 9;   // Intake rollers PWM channel (Spark)
    constexpr double kMotorReverseConstant = 1;

    constexpr double kIngestLow = 0.3;
    constexpr double kIngestHigh = 0.78;
    constexpr double kReleaseLow = -0.3;
    constexpr double kReleaseHigh = -0.70;
}

// Flywheel Subsystem constants
namespace FlywheelConstants
{
    constexpr double kMotorPort = 20;       //!< Flywheel CAN ID (SparkMAX)

    constexpr double kRampRate = 1.0;
    // Total error allowed for the flywheel, in RPM
    constexpr double kAllowedError = 50;
    constexpr double kMaintainPIDError = 300;

    // Additional multiplier applied to flywheel speed while firing 
    // Ensures all ball trajectories are straight
    constexpr double kFiringRPMMultiplier = 1.05;

    // Launch PID values, used to first get to setpoint
    constexpr double kP = 0.0002700;
    constexpr double kI = 0;
    constexpr double kD = 0;

    // Maintain PID values, used to adjust for error once the robot is shooting
    constexpr double kMP = 0.001700;
    constexpr double kMI = 0.00000001;
    constexpr double kMD = 0.000001;

    constexpr double kMinOut = 0;
    constexpr double kMaxOut = 1.0;

    constexpr double kS = 0.059;
    constexpr double kV = 0.102834;
    constexpr double kA = 0;//0.0399;

    // Diameter is in meters
    constexpr double kWheelDiameter = 0.1524;
    constexpr double kSecondsPerMinute = 60;
    constexpr double kWheelMetersPerRev = kWheelDiameter * wpi::math::pi;
    // Meters per second to Revolutions per minute
    constexpr double kMPSPerRPM = kWheelMetersPerRev / kSecondsPerMinute;
    constexpr double kWheelRevPerMotorRev = 1.25;

    /// Use MPSPerRPM to determine the ramp rates, current values are just placeholders
    constexpr double kIdleRPM = 0; //1700;
}

// Turret Subsystem Constants
namespace TurretConstants
{
    constexpr double kMotorPort = 11;   //!< Turret CAN ID (TalonSRX)

    constexpr double kP = 0.32114;
    constexpr double kI = 0.00035;
    constexpr double kD = 19.6;

    constexpr double kMinOut = 0;
    constexpr double kMaxOut = 0.700;

    constexpr double kTimeout = 30;
    constexpr double kInverted = true;
    constexpr double kSensorPhase = true;

    constexpr double kDegreeStopRange = 0.5;
    constexpr double kDegreePIDStopRange = 0.35;

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
    constexpr double kMaxAngle = 90;

    // initial configured angle of the turret relative to the turret, in degrees
    constexpr double kStartingPositionDegrees = 45;
}

/// Hood subsystem constants
namespace HoodConstants
{
    /// PWM Port for hood servo
    constexpr int kPWMPort = 8;                //!< Hood servo PWM channel
    constexpr double kTestServoSpeed = 0.14;
    // Drives from Max to Min, where hood is smallest at 0.85, and greatest at 0.0485
    constexpr double kMax = .9;
    constexpr double kMin = .20;
}

// Cycler Subsystem Constants
namespace CyclerConstants
{
    constexpr double kFeederPort = 30;      //!< Feeder CAN ID (SparkMAX)
    constexpr double kTurnTablePort = 31;   //!< Turn table CAN ID (TalonSRX)

    constexpr double kFeederSpeed = 0.350;
    constexpr double kTurnTableSpeed = 0.400;
    constexpr double kTurnTableSpeedHigher = 0.550;
    constexpr double kTurnTableHoneSpeed = 0.200;
    constexpr units::second_t kMaxCyclerTime = 5.0_s;

    constexpr double kSensorInvert = true;

    // Time to go from 0 to full throttle
    constexpr double kTurnTableRampRate = 0.75;

    constexpr double kTimePassed = 0.25;
    constexpr double kTimeLaunch = 4.00;

    constexpr double kTimeout = 30;
    constexpr double kTurnTableInverted = false;
    constexpr double kFeederInverted = true;
}

// Vision Subsystem Constants
namespace VisionConstants
{
    // Mounting angle of the limelight, in degrees
    constexpr double kMountingAngle = 19.0;
    // Mounting height of the limelight from the ground, in inches
    constexpr double kMountingHeight = 22;
    // Target center height, in inches
    constexpr double kTargetHeight = 98.25;
    // Target width, in inches
    constexpr double kTargetSize = 15;

    constexpr double kMinTargetDistance = 70;
    constexpr double kMaxTargetDistance = 380;
}

// Climber Subsystem constants
namespace ClimberConstants
{
    constexpr double kMotorPort = 41;   // Climber CAN ID (TalonSRX? not installed)
    constexpr double kMotorReverseConstant = 1;

}