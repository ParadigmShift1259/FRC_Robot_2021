/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/units.h>
#include <wpi/math>

#include "Constants.h"
#include <iostream>
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace DriveConstants;
using namespace std;
using namespace frc;

DriveSubsystem::DriveSubsystem(Logger& log)
    : m_log(log)
    , m_logData(c_headerNamesDriveSubsystem, true, "")
    , m_frontLeft
      {
          kFrontLeftDriveMotorPort
        , kFrontLeftTurningMotorPort
        , kFrontLeftDriveEncoderPort
        , kFrontLeftTurningEncoderPort
        , kFrontLeftDriveMotorReversed
        , kFrontLeftTurningEncoderReversed
        , kFrontLeftOffset
        , std::string("FrontLeft")
        , log
      }

    , m_frontRight
      {
          kFrontRightDriveMotorPort
        , kFrontRightTurningMotorPort
        , kFrontRightDriveEncoderPort
        , kFrontRightTurningEncoderPort
        , kFrontRightDriveMotorReversed
        , kFrontRightTurningEncoderReversed
        , kFrontRightOffset
        , std::string("FrontRight")
        , log
      }

    , m_rearRight
      {
          kRearRightDriveMotorPort
        , kRearRightTurningMotorPort
        , kRearRightDriveEncoderPort
        , kRearRightTurningEncoderPort
        , kRearRightDriveMotorReversed
        , kRearRightTurningEncoderReversed
        , kRearRightOffset
        , std::string("RearRight")
        , log
      }

    , m_rearLeft
      {
          kRearLeftDriveMotorPort
        , kRearLeftTurningMotorPort
        , kRearLeftDriveEncoderPort
        , kRearLeftTurningEncoderPort
        , kRearLeftDriveMotorReversed
        , kRearLeftTurningEncoderReversed
        , kRearLeftOffset
        , std::string("RearLeft")
        , log
      }

    , m_gyro(0)
    , m_odometry{kDriveKinematics, GetHeadingAsRot2d(), frc::Pose2d()}
{
    SmartDashboard::PutBoolean("GetInputFromNetTable", true);

    SmartDashboard::PutNumber("FrontLeft", 0.0);
    SmartDashboard::PutNumber("FrontRight", 0.0);
    SmartDashboard::PutNumber("RearRight", 0.0);
    SmartDashboard::PutNumber("RearLeft", 0.0);
    
    SmartDashboard::PutNumber("FrontLeftV", 0.0);
    SmartDashboard::PutNumber("FrontRightV", 0.0);
    SmartDashboard::PutNumber("RearLeftV", 0.0);
    SmartDashboard::PutNumber("RearRightV", 0.0);

    SmartDashboard::PutNumber("kP", ModuleConstants::kP_ModuleTurningController);
    SmartDashboard::PutNumber("kd", ModuleConstants::kD_ModuleTurningController);
    SmartDashboard::PutNumber("kI", 0.000);

    SmartDashboard::PutNumber("kP drive", ModuleConstants::kPModuleDriveController);

    SmartDashboard::PutNumber("Tolerance", 0.1);
}

void DriveSubsystem::Periodic()
{
    // Implementation of subsystem periodic method goes here.
    m_odometry.Update(GetHeadingAsRot2d()
                    , m_frontLeft.GetState()
                    , m_rearLeft.GetState() // TODO check order FL, RL?
                    , m_frontRight.GetState()
                    , m_rearRight.GetState());
   
    auto pose = m_odometry.GetPose();

    m_logData[EDriveSubSystemLogData::eOdoX] = pose.Translation().X().to<double>();
    m_logData[EDriveSubSystemLogData::eOdoY] = pose.Translation().Y().to<double>();
    m_logData[EDriveSubSystemLogData::eOdoRot] = pose.Rotation().Degrees().to<double>();
    m_log.logData<EDriveSubSystemLogData>("DriveSubsystem::Periodic", __LINE__, m_logData);

    m_frontLeft.Periodic();
    m_frontRight.Periodic();
    m_rearRight.Periodic();
    m_rearLeft.Periodic();
}

void DriveSubsystem::RotationDrive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, double xRot, double yRot, bool fieldRelative) {
    radian_t rotPosition = radian_t(atan2f(yRot, xRot));
    double rotDifference = rotPosition.to<double>() - GetHeadingAsRot2d().Radians().to<double>();
    double theta = rotDifference;
    if (theta > wpi::math::pi)
        theta -= 2 * wpi::math::pi;
    else if (theta < -1.0 * wpi::math::pi)
        theta += 2 * wpi::math::pi;
    
    SmartDashboard::PutNumber("TEST_Rotation Difference", theta);
    Drive(xSpeed, ySpeed, radians_per_second_t(theta * 0.5), fieldRelative);
}

void DriveSubsystem::Drive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rot, bool fieldRelative)
{
    m_logData[EDriveSubSystemLogData::eInputX] = xSpeed.to<double>();
    m_logData[EDriveSubSystemLogData::eInputY] = ySpeed.to<double>();
    m_logData[EDriveSubSystemLogData::eInputRot] = rot.to<double>();

    // if (xSpeed.to<double>() == 0.0 && ySpeed.to<double>() == 0.0 && rot.to<double>() == 0.0)
    // {
    //     states[eFrontLeft].angle = frc::Rotation2d(radian_t(0.0));
    //     states[eFrontRight].angle = frc::Rotation2d(radian_t(0.0));
    //     states[eRearLeft].angle = frc::Rotation2d(radian_t(0.0));
    //     states[eRearRight].angle = frc::Rotation2d(radian_t(0.0));
    // }

    frc::ChassisSpeeds chassisSpeeds;
    if (fieldRelative)
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeadingAsRot2d());
    else
        chassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, rot};

    auto states = kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);

    kDriveKinematics.NormalizeWheelSpeeds(&states, AutoConstants::kMaxSpeed);
    
    //if (SmartDashboard::GetBoolean("GetInputFromNetTable", false))
    if (false)
    {
        double angle = SmartDashboard::GetNumber("FrontLeft", 0.0);
        states[eFrontLeft].angle = frc::Rotation2d(radian_t(angle));

        angle = SmartDashboard::GetNumber("FrontRight", 0.0);
        states[eFrontRight].angle = frc::Rotation2d(radian_t(angle));

        angle = SmartDashboard::GetNumber("RearLeft", 0.0);
        states[eRearLeft].angle = frc::Rotation2d(radian_t(angle));

        angle = SmartDashboard::GetNumber("RearRight", 0.0);
        states[eRearRight].angle = frc::Rotation2d(radian_t(angle));

        double speed = SmartDashboard::GetNumber("FrontLeftV", 0.0);
        states[eFrontLeft].speed = meters_per_second_t(speed);

        speed = SmartDashboard::GetNumber("FrontRightV", 0.0);
        states[eFrontLeft].speed = meters_per_second_t(speed);

        speed = SmartDashboard::GetNumber("RearLeftV", 0.0);
        states[eFrontLeft].speed = meters_per_second_t(speed);

        speed = SmartDashboard::GetNumber("RearRightV", 0.0);
        states[eFrontLeft].speed = meters_per_second_t(speed);
    }

    m_frontLeft.SetDesiredState(states[eFrontLeft]);
    m_frontRight.SetDesiredState(states[eFrontRight]);
    m_rearRight.SetDesiredState(states[eRearRight]);
    m_rearLeft.SetDesiredState(states[eRearLeft]);
}

void DriveSubsystem::SetModuleStates(SwerveModuleStates desiredStates)
{
    kDriveKinematics.NormalizeWheelSpeeds(&desiredStates, AutoConstants::kMaxSpeed);
    m_frontLeft.SetDesiredState(desiredStates[eFrontLeft]);
    m_frontRight.SetDesiredState(desiredStates[eFrontRight]);
    m_rearRight.SetDesiredState(desiredStates[eRearRight]);
    m_rearLeft.SetDesiredState(desiredStates[eRearLeft]);
}

void DriveSubsystem::ResetEncoders()
{
    m_frontLeft.ResetEncoders();
    m_frontRight.ResetEncoders();
    m_rearRight.ResetEncoders();
    m_rearLeft.ResetEncoders();
}

double DriveSubsystem::GetHeading()
{
    auto retVal = std::remainder(m_gyro.GetFusedHeading(), 360.0) * (kGyroReversed ? -1. : 1.);
    if (retVal > 180.0)
    {
        retVal -= 360.0;
    }

    SmartDashboard::PutNumber("Rotation", retVal);

    return retVal;
}

void DriveSubsystem::ZeroHeading()
{
    m_gyro.ClearStickyFaults();
    //m_gyro.SetFusedHeading(0.0, 0);
}

double DriveSubsystem::GetTurnRate()
{
    return 0.0; //m_gyro.GetRate() * (kGyroReversed ? -1. : 1.);
}

frc::Pose2d DriveSubsystem::GetPose()
{
    return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
    m_odometry.ResetPosition(pose, GetHeadingAsRot2d());
}
