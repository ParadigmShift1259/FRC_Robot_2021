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

DriveSubsystem::DriveSubsystem(Logger& log, const int& lowPrioritySkipCount)
    : m_log(log)
    , m_logData(c_headerNamesDriveSubsystem, false, "")
    , m_frontLeft
      {
          kFrontLeftDriveMotorPort
        , kFrontLeftTurningMotorPort
#ifdef Mk2
        , kFrontLeftTurningEncoderPort
#else
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kFrontLeftPWM
#endif
        , kFrontLeftDriveMotorReversed
        , kFrontLeftOffset
        , std::string("FrontLeft")
        , log
      }

    , m_frontRight
      {
          kFrontRightDriveMotorPort
        , kFrontRightTurningMotorPort
#ifdef Mk2
        , kFrontRightTurningEncoderPort
#else
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kFrontRightPWM
#endif
        , kFrontRightDriveMotorReversed
        , kFrontRightOffset
        , std::string("FrontRight")
        , log
      }

    , m_rearRight
      {
          kRearRightDriveMotorPort
        , kRearRightTurningMotorPort
#ifdef Mk2
        , kRearRightTurningEncoderPort
#else
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kRearRightPWM
#endif
        , kRearRightDriveMotorReversed
        , kRearRightOffset
        , std::string("RearRight")
        , log
      }

    , m_rearLeft
      {
          kRearLeftDriveMotorPort
        , kRearLeftTurningMotorPort
#ifdef Mk2
        , kRearLeftTurningEncoderPort
#else
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kRearLeftPWM
#endif
        , kRearLeftDriveMotorReversed
        , kRearLeftOffset
        , std::string("RearLeft")
        , log
      }
    , m_canifier(DriveConstants::kCanifierID)
    , m_gyro(0)
    , m_odometry{kDriveKinematics, GetHeadingAsRot2d(), frc::Pose2d()}
    , m_lowPrioritySkipCount(lowPrioritySkipCount)
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

    m_frontLeft.Periodic(m_lowPrioritySkipCount);
    m_frontRight.Periodic(m_lowPrioritySkipCount);
    m_rearRight.Periodic(m_lowPrioritySkipCount);
    m_rearLeft.Periodic(m_lowPrioritySkipCount);

    if (m_lowPrioritySkipCount % 10 == 0)   // 5 per second
    {
        m_logData[EDriveSubSystemLogData::eOdoX] = pose.Translation().X().to<double>();
        m_logData[EDriveSubSystemLogData::eOdoY] = pose.Translation().Y().to<double>();
        m_logData[EDriveSubSystemLogData::eOdoRot] = pose.Rotation().Degrees().to<double>();
        m_logData[EDriveSubSystemLogData::eGyroRot] = GetHeading();
        m_logData[EDriveSubSystemLogData::eGyroRotRate] = GetTurnRate();
        m_log.logData<EDriveSubSystemLogData>("DriveSubsys", m_logData);

        SmartDashboard::PutNumber("D_D_Rot", GetHeading());
    }
}

void DriveSubsystem::RotationDrive(meters_per_second_t xSpeed
                                , meters_per_second_t ySpeed
                                , double xRot
                                , double yRot
                                , bool fieldRelative) 
{  
    if (xRot != 0 || yRot != 0)
	{
        double rotPosition = atan2f(yRot, xRot);

        double error = rotPosition - GetHeadingAsRot2d().Radians().to<double>();
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

        if (!m_rotationPIDController.AtSetpoint())
            Drive(xSpeed, ySpeed, radians_per_second_t(desiredTurnRate), fieldRelative);
        else
            Drive(xSpeed, ySpeed, radians_per_second_t(0), fieldRelative);
    }
    else
        Drive(xSpeed, ySpeed, radians_per_second_t(0), fieldRelative);
}

void DriveSubsystem::Drive(meters_per_second_t xSpeed
                        , meters_per_second_t ySpeed
                        , radians_per_second_t rot
                        , bool fieldRelative)
{
    m_logData[EDriveSubSystemLogData::eInputX] = xSpeed.to<double>();
    m_logData[EDriveSubSystemLogData::eInputY] = ySpeed.to<double>();
    m_logData[EDriveSubSystemLogData::eInputRot] = rot.to<double>();

    frc::ChassisSpeeds chassisSpeeds;
    if (fieldRelative)
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeadingAsRot2d());
    else
        chassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, rot};

    auto states = kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);

    kDriveKinematics.NormalizeWheelSpeeds(&states, AutoConstants::kMaxSpeed);
    
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
