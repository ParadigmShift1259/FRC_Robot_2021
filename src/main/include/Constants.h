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

    /// \name Drive wheel reversal (inverting) flags
    ///@{
    /// To keep the swerve module bevel gear facing inwards we need to reverse the right side
    constexpr bool kFrontLeftDriveMotorReversed  = false;
    constexpr bool kRearLeftDriveMotorReversed   = false;
    constexpr bool kFrontRightDriveMotorReversed = true;
    constexpr bool kRearRightDriveMotorReversed  = true;
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

    // constexpr double kFrontLeftOffset   = 3.142; //6.412;           //3.142;         // 3.14;
    // constexpr double kFrontRightOffset  = 5.105; //5.155 + 1.57;    //5.105;         // 5.07;         //5.66;
    // constexpr double kRearLeftOffset    = 1.6292; //1.8292; //4.85;       //1.42921;       // 3.34;         //4.29;
    // constexpr double kRearRightOffset   = 0.665; //0.635 + 1.57;    //0.665;         // 0.63;         //5.29;

    // constexpr double kFrontLeftOffset   = 3.142;
    // constexpr double kFrontRightOffset  = 5.105 - wpi::math::pi;
    // constexpr double kRearLeftOffset    = 1.8292;
    // constexpr double kRearRightOffset   = 0.665 + wpi::math::pi;

    // Mk3 swerve module
    constexpr double kFrontLeftOffset   = 2685.0; // 2.163;
    constexpr double kFrontRightOffset  =  251.0; // 5.897;
    constexpr double kRearRightOffset   = 1875.0; // 3.405;
    constexpr double kRearLeftOffset    = 3867.0; // 0.351;

    // For MK2 swerve encoders
    //constexpr double kMaxAnalogVoltage = 4.93;                              //!< Absolute encoder runs 0 to 4.93V
    //constexpr double kTurnVoltageToRadians = Math::kTau / kMaxAnalogVoltage;
    //constexpr double KTurnVoltageToDegrees = 360 / kMaxAnalogVoltage;

    constexpr double kPulseWidthToZeroOne = 4096.0;    // 4096 micro second pulse width is full circle
    constexpr double kPulseWidthToRadians =  Math::kTau / kPulseWidthToZeroOne;
    //constexpr double kPulseWidthToDegrees = 360 / kPulseWidthToZeroOne;

     // Rotation PID Controller, converts between radians angle error to radians per second turn

    /// Used to regulate PID on turning (especially field relative turning)
    constexpr double kTurnValidationDistance = 0.35;

    /// Turn PID Controller for Swerve Modules
    constexpr double kTurnP = 0.0001;   //0.35;
    constexpr double kTurnI = 0.0;      //1e-4;
    constexpr double kTurnD = 0.2;      //1.0; // 1.85

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

}  // namespace DriveConstants

namespace ModuleConstants
{
    constexpr int kEncoderCPR = 2048;
    constexpr int kEncoderTicksPerSec = 10;                 //!< TalonFX::GetSelectedSensorVelocity() returns ticks/100ms = 10 ticks/sec
    constexpr double kWheelDiameterMeters = .1016;          //!< 4"
    //constexpr double kDriveGearRatio = 8.31;              //!< MK2 swerve modules 11.9 ft/sec
    constexpr double kDriveGearRatio = 8.16;                //!< MK3 swerve modules w/NEOs 12.1 ft/sec w/Falcon 13.6 ft/sec
    //constexpr double kDriveGearRatio = 6.86;              //!< MK3 swerve modules w/NEOs 14.4 ft/sec
    constexpr double kTurnMotorRevsPerWheelRev = 12.8;//18.0;
    /// Assumes the encoders are directly mounted on the wheel shafts
    constexpr double kDriveEncoderMetersPerSec = (kWheelDiameterMeters * wpi::math::pi * kDriveGearRatio) / static_cast<double>(kEncoderCPR);

    constexpr double kTurnEncoderCPR = 4096.0 * kTurnMotorRevsPerWheelRev;    // Mag encoder relative output to SparkMax

    constexpr double kP_ModuleTurningController = 0.01;//1.1;
    constexpr double kD_ModuleTurningController = 0.03;

    constexpr double kPModuleDriveController = 0.001;

    constexpr uint kMotorCurrentLimit = 30;
}   // namespace ModuleConstants

namespace AutoConstants
{
    using radians_per_second_squared_t = units::compound_unit<units::radians, units::inverse<units::squared<units::second>>>;

    constexpr auto kMaxSpeed = units::meters_per_second_t(0.7); // units::meters_per_second_t(5.0);
    constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(1.0);
    constexpr auto kMaxAngularSpeed = units::radians_per_second_t(wpi::math::pi);
    constexpr auto kMaxAngularAcceleration = units::unit_t<radians_per_second_squared_t>(wpi::math::pi);

    constexpr double kPXController = 0.25;
    constexpr double kPYController = 0.25;
    constexpr double kPThetaController = 0.5;

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
