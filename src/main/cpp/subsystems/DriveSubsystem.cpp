/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace DriveConstants;
using namespace std;
using namespace frc;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft
      {
          kFrontLeftDriveMotorPort
        , kFrontLeftTurningMotorPort
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kFrontLeftPWM
        , kFrontLeftDriveMotorReversed
        , kFrontLeftOffset
        , std::string("FrontLeft")
      }
    , m_frontRight
      {
          kFrontRightDriveMotorPort
        , kFrontRightTurningMotorPort
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kFrontRightPWM
        , kFrontRightDriveMotorReversed
        , kFrontRightOffset
        , std::string("FrontRight")
      }
    , m_rearRight
      {
          kRearRightDriveMotorPort
        , kRearRightTurningMotorPort
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kRearRightPWM
        , kRearRightDriveMotorReversed
        , kRearRightOffset
        , std::string("RearRight")
      }
    , m_rearLeft
      {
          kRearLeftDriveMotorPort
        , kRearLeftTurningMotorPort
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kRearLeftPWM
        , kRearLeftDriveMotorReversed
        , kRearLeftOffset
        , std::string("RearLeft")
      }
    , m_canifier(DriveConstants::kCanifierID)
    , m_gyro(0)
    , m_odometry{kDriveKinematics, GetHeadingAsRot2d(), frc::Pose2d()}
{

    #ifdef TUNE_MODULES
    SmartDashboard::PutNumber("T_D_MFL", 0);
    SmartDashboard::PutNumber("T_D_MFR", 0);
    SmartDashboard::PutNumber("T_D_MRR", 0);
    SmartDashboard::PutNumber("T_D_MRL", 0);
    SmartDashboard::PutNumber("T_D_MFLV", 0);
    SmartDashboard::PutNumber("T_D_MFRV", 0);
    SmartDashboard::PutNumber("T_D_MRRV", 0);
    SmartDashboard::PutNumber("T_D_MRLV", 0);
    #endif

    #ifdef TUNE_ROTATION
    SmartDashboard::PutNumber("T_D_RP", 0);
    SmartDashboard::PutNumber("T_D_RI", 0);
    SmartDashboard::PutNumber("T_D_RD", 0);
    SmartDashboard::PutNumber("T_D_RMax", 0);
    SmartDashboard::PutNumber("T_D_RTMax", 0);
    #endif

    m_rotationPIDController.SetTolerance(DriveConstants::kAbsoluteRotationTolerance);
    m_rotationPIDController.SetIntegratorRange(0, DriveConstants::kRotationIMaxRange);

    m_lastHeading = 0;
    m_rotationalInput = true;
}

void DriveSubsystem::Periodic()
{
    // Implementation of subsystem periodic method goes here.
    m_odometry.Update(GetHeadingAsRot2d()
                    , m_frontLeft.GetState()
                    , m_frontRight.GetState()
                    , m_rearLeft.GetState()
                    , m_rearRight.GetState());
   
    auto pose = m_odometry.GetPose();

    m_frontLeft.Periodic();
    m_frontRight.Periodic();
    m_rearRight.Periodic();
    m_rearLeft.Periodic();
    SmartDashboard::PutNumber("D_D_Rot", GetHeading());
}

void DriveSubsystem::RotationDrive(meters_per_second_t xSpeed
                                , meters_per_second_t ySpeed
                                , radian_t rot
                                , bool fieldRelative) 
{  
    double error = rot.to<double>() - GetHeadingAsRot2d().Radians().to<double>();
    double desiredSet = Util::NegPiToPiRads(error);
    double max = DriveConstants::kMaxAbsoluteRotationSpeed;
    double maxTurn = DriveConstants::kMaxAbsoluteTurnableSpeed;

    #ifdef TUNE_ROTATION
    double P = SmartDashboard::GetNumber("T_D_RP", 0);
    double I = SmartDashboard::GetNumber("T_D_RI", 0);
    double D = SmartDashboard::GetNumber("T_D_RD", 0);
    double m = SmartDashboard::GetNumber("T_D_RMax", 0);
    double mTurn = SmartDashboard::GetNumber("T_D_RTMax", 0);

    m_rotationPIDController.SetP(P);
    m_rotationPIDController.SetI(I);
    m_rotationPIDController.SetD(D);
    max = m;
    maxTurn = mTurn;
    #endif

    double desiredTurnRate = m_rotationPIDController.Calculate(0, desiredSet);

    double currentTurnRate = GetTurnRate() * wpi::math::pi / 180;

    // Prevent sharp turning if already fast going in a direction
    if ((abs(currentTurnRate) >= maxTurn) && (signbit(desiredTurnRate) != signbit(currentTurnRate)))
        desiredTurnRate *= -1.0;

    // Power limiting
    if (abs(desiredTurnRate) > max)
        desiredTurnRate = signbit(desiredTurnRate) ? max * -1.0 : max;

    // if (!m_rotationPIDController.AtSetpoint())
        Drive(xSpeed, ySpeed, radians_per_second_t(desiredTurnRate), fieldRelative);
    // else
    //     Drive(xSpeed, ySpeed, radians_per_second_t(0), fieldRelative);
}

void DriveSubsystem::RotationDrive(meters_per_second_t xSpeed
                                , meters_per_second_t ySpeed
                                , double xRot
                                , double yRot
                                , bool fieldRelative) 
{
    if (xRot != 0 || yRot != 0)
    {
        m_rotationalInput = true;
        RotationDrive(xSpeed, ySpeed, radian_t(atan2f(yRot, xRot)), fieldRelative);
    }
    else
        Drive(xSpeed, ySpeed, radians_per_second_t(0), fieldRelative);
    
}

void DriveSubsystem::HeadingDrive(meters_per_second_t xSpeed
                        , meters_per_second_t ySpeed
                        , radians_per_second_t rot
                        , bool fieldRelative)
{
    if (rot.to<double>() == 0 && m_rotationalInput)
    {
        m_rotationalInput = false;
        m_lastHeading = GetHeadingAsRot2d().Radians().to<double>();
    }
    else
    if (rot.to<double>() != 0)
        m_rotationalInput = true;
    
    if (!m_rotationalInput)
        RotationDrive(xSpeed, ySpeed, radian_t(m_lastHeading), fieldRelative);
    else
        Drive(xSpeed, ySpeed, rot, fieldRelative);
}

void DriveSubsystem::Drive(meters_per_second_t xSpeed
                        , meters_per_second_t ySpeed
                        , radians_per_second_t rot
                        , bool fieldRelative)
{
    frc::ChassisSpeeds chassisSpeeds;
    if (fieldRelative)
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeadingAsRot2d());
    else
        chassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, rot};

    auto states = kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);

    kDriveKinematics.NormalizeWheelSpeeds(&states, AutoConstants::kMaxSpeed);

    // Added to force correct slow left side, 3/24/21
    states[eFrontLeft].speed *= DriveConstants::kLeftMultipler;
    states[eRearLeft].speed *= DriveConstants::kLeftMultipler;
    
    #ifdef TUNE_MODULES
    states[eFrontLeft].angle = frc::Rotation2d(radian_t(SmartDashboard::GetNumber("T_D_MFL", 0.0)));
    states[eFrontRight].angle = frc::Rotation2d(radian_t(SmartDashboard::GetNumber("T_D_MFR", 0.0)));
    states[eRearRight].angle = frc::Rotation2d(radian_t(SmartDashboard::GetNumber("T_D_MRR", 0.0)));
    states[eRearLeft].angle = frc::Rotation2d(radian_t(SmartDashboard::GetNumber("T_D_MRL", 0.0)));
    states[eFrontLeft].speed = SmartDashboard::GetNumber("T_D_MFLV", 0.0) * 1_mps;
    states[eFrontRight].speed = SmartDashboard::GetNumber("T_D_MFRV", 0.0) * 1_mps;
    states[eRearRight].speed = SmartDashboard::GetNumber("T_D_MRRV", 0.0) * 1_mps;
    states[eRearLeft].speed = SmartDashboard::GetNumber("T_D_MRLV", 0.0) * 1_mps;
    #endif

    m_frontLeft.SetDesiredState(states[eFrontLeft]);
    m_frontRight.SetDesiredState(states[eFrontRight]);
    m_rearLeft.SetDesiredState(states[eRearLeft]);
    m_rearRight.SetDesiredState(states[eRearRight]);
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
        retVal -= 360.0;

    return retVal;
}

void DriveSubsystem::ZeroHeading()
{
    m_gyro.SetFusedHeading(0.0, 0);
}

double DriveSubsystem::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    m_gyro.GetRawGyro(turnRates);
    return turnRates[2] * (kGyroReversed ? -1. : 1.); 
}

frc::Pose2d DriveSubsystem::GetPose()
{
    SmartDashboard::PutNumber("T_D_Odo_X", m_odometry.GetPose().X().to<double>());
    SmartDashboard::PutNumber("T_D_Odo_Y", m_odometry.GetPose().Y().to<double>());
    SmartDashboard::PutNumber("T_D_Odo_Angle", m_odometry.GetPose().Rotation().Degrees().to<double>());
    return m_odometry.GetPose();
}

double DriveSubsystem::PWMToPulseWidth(CANifier::PWMChannel pwmChannel)
{
    double dutyCycleAndPeriod[2];
    m_canifier.GetPWMInput(pwmChannel, dutyCycleAndPeriod);
    return dutyCycleAndPeriod[0] * dutyCycleAndPeriod[1] / kPulseWidthToZeroOne;
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
    m_odometry.ResetPosition(pose, GetHeadingAsRot2d());
}

void DriveSubsystem::ResetRelativeToAbsolute()
{
    m_frontLeft.ResetRelativeToAbsolute();
    m_frontRight.ResetRelativeToAbsolute();
    m_rearRight.ResetRelativeToAbsolute();
    m_rearLeft.ResetRelativeToAbsolute();
}