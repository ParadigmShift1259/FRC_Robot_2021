/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace DriveConstants;
using namespace std;
using namespace frc;

DriveSubsystem::DriveSubsystem(Logger& log)
    : m_log(log)
    , m_logData(c_headerNamesDriveSubsystem, false, "")
    , m_frontLeft
      {
          kFrontLeftDriveMotorPort
        , kFrontLeftTurningMotorPort
        , kFrontLeftTurningEncoderPort
        , kFrontLeftDriveMotorReversed
        , kFrontLeftOffset
        , std::string("FrontLeft")
        , log
      }

    , m_frontRight
      {
          kFrontRightDriveMotorPort
        , kFrontRightTurningMotorPort
        , kFrontRightTurningEncoderPort
        , kFrontRightDriveMotorReversed
        , kFrontRightOffset
        , std::string("FrontRight")
        , log
      }

    , m_rearRight
      {
          kRearRightDriveMotorPort
        , kRearRightTurningMotorPort
        , kRearRightTurningEncoderPort
        , kRearRightDriveMotorReversed
        , kRearRightOffset
        , std::string("RearRight")
        , log
      }

    , m_rearLeft
      {
          kRearLeftDriveMotorPort
        , kRearLeftTurningMotorPort
        , kRearLeftTurningEncoderPort
        , kRearLeftDriveMotorReversed
        , kRearLeftOffset
        , std::string("RearLeft")
        , log
      }

    , m_gyro(0)
    , m_odometry{kDriveKinematics, GetHeadingAsRot2d(), frc::Pose2d()}
{
    //SmartDashboard::PutBoolean("GetInputFromNetTable", false);

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

    /*
    SmartDashboard::PutNumber("kMaxRotation", DriveConstants::kMaxAbsoluteRotationSpeed);
    SmartDashboard::PutNumber("kMaxRotationSpeed", DriveConstants::kMaxAbsoluteTurnableSpeed);
    SmartDashboard::PutNumber("kRotationP", DriveConstants::kRotationP);
    SmartDashboard::PutNumber("kRotationI", DriveConstants::kRotationI);
    SmartDashboard::PutNumber("kRotationD", DriveConstants::kRotationD);
    */

    SmartDashboard::PutNumber("kP drive", ModuleConstants::kPModuleDriveController);

    SmartDashboard::PutNumber("Tolerance", 0.1);

    m_rotationPIDController.SetTolerance(DriveConstants::kAbsoluteRotationTolerance);
    m_rotationPIDController.SetIntegratorRange(0, DriveConstants::kRotationIMaxRange);
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
    m_logData[EDriveSubSystemLogData::eGyroRot] = GetHeading();
    m_logData[EDriveSubSystemLogData::eGyroRotRate] = GetTurnRate() * wpi::math::pi / 180;
    m_log.logData<EDriveSubSystemLogData>("DriveSubsys", m_logData);

    m_frontLeft.Periodic();
    m_frontRight.Periodic();
    m_rearRight.Periodic();
    m_rearLeft.Periodic();
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
        double desiredSet = SwerveModule::NegPiToPiRads(error);

        /*
        SmartDashboard::PutNumber("TEST_error", error);
        SmartDashboard::PutNumber("TEST_yRot", yRot);
        SmartDashboard::PutNumber("TEST_xRot", xRot);
        SmartDashboard::PutNumber("TEST_RotationPosition", rotPosition);
        SmartDashboard::PutNumber("TEST_ActualPosition", GetHeadingAsRot2d().Radians().to<double>());
        */

                                                                    // Used for tuning RotationDrive PID
        double max = DriveConstants::kMaxAbsoluteRotationSpeed;     //SmartDashboard::GetNumber("kMaxRotation", 0);
        double maxTurn = DriveConstants::kMaxAbsoluteTurnableSpeed; //SmartDashboard::GetNumber("kMaxRotationSpeed", 0);
        double P = DriveConstants::kRotationP;                      //SmartDashboard::GetNumber("kRotationP", 0);
        double I = DriveConstants::kRotationI;                      //SmartDashboard::GetNumber("kRotationI", 0);
        double D = DriveConstants::kRotationD;                      //SmartDashboard::GetNumber("kRotationD", 0);

        /* Used for tuning RotationDrive PID
        m_rotationPIDController.SetP(P);
        m_rotationPIDController.SetI(I);
        m_rotationPIDController.SetD(D);
        */

        double desiredTurnRate = m_rotationPIDController.Calculate(0, desiredSet);

        double currentTurnRate = GetTurnRate() * wpi::math::pi / 180;

        // Prevent sharp turning if already fast going in a direction
        //SmartDashboard::PutNumber("TEST_Turn Rate", currentTurnRate);
        if ((abs(currentTurnRate) >= maxTurn) && (signbit(desiredTurnRate) != signbit(currentTurnRate)))
        {
            desiredTurnRate *= -1.0;
        }

        // Power limiting
        if (abs(desiredTurnRate) > max)
        {
            desiredTurnRate = signbit(desiredTurnRate) ? max * -1.0 : max;
        }
        
        //SmartDashboard::PutNumber("TEST_Rotation Difference", desiredSet);
        //SmartDashboard::PutNumber("TEST_Rotation Power (-1 -> 1)", desiredTurnRate);

        if (!m_rotationPIDController.AtSetpoint())
		{
            Drive(xSpeed, ySpeed, radians_per_second_t(desiredTurnRate), fieldRelative);
        }
        else
		{
            Drive(xSpeed, ySpeed, radians_per_second_t(0), fieldRelative);
        }
    }
    else
    {
        Drive(xSpeed, ySpeed, radians_per_second_t(0), fieldRelative);
    }
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
    
    //if (SmartDashboard::GetBoolean("GetInputFromNetTable", false))
    // if (false)
    // {
    //     double angle = SmartDashboard::GetNumber("FrontLeft", 0.0);
    //     states[eFrontLeft].angle = frc::Rotation2d(radian_t(angle));

    //     angle = SmartDashboard::GetNumber("FrontRight", 0.0);
    //     states[eFrontRight].angle = frc::Rotation2d(radian_t(angle));

    //     angle = SmartDashboard::GetNumber("RearLeft", 0.0);
    //     states[eRearLeft].angle = frc::Rotation2d(radian_t(angle));

    //     angle = SmartDashboard::GetNumber("RearRight", 0.0);
    //     states[eRearRight].angle = frc::Rotation2d(radian_t(angle));

    //     double speed = SmartDashboard::GetNumber("FrontLeftV", 0.0);
    //     states[eFrontLeft].speed = meters_per_second_t(speed);

    //     speed = SmartDashboard::GetNumber("FrontRightV", 0.0);
    //     states[eFrontLeft].speed = meters_per_second_t(speed);

    //     speed = SmartDashboard::GetNumber("RearLeftV", 0.0);
    //     states[eFrontLeft].speed = meters_per_second_t(speed);

    //     speed = SmartDashboard::GetNumber("RearRightV", 0.0);
    //     states[eFrontLeft].speed = meters_per_second_t(speed);
    // }

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
    //m_gyro.ClearStickyFaults();
    m_gyro.SetFusedHeading(0.0, 0);
}

double DriveSubsystem::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    m_gyro.GetRawGyro(turnRates) * (kGyroReversed ? -1. : 1.);
    SmartDashboard::PutNumber("TurnRate", turnRates[2]);
    return turnRates[2]; 
}

frc::Pose2d DriveSubsystem::GetPose()
{
    // TODO needed? m_odometry.UpdateWithTime(m_timer.Get(), m_angle, getCurrentWheelSpeeds());
    return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
    m_odometry.ResetPosition(pose, GetHeadingAsRot2d());
}

void DriveSubsystem::WheelsForward()
{
    printf("WheelsForward\n");
    m_frontLeft.WheelForward();
    m_frontRight.WheelForward();
    m_rearRight.WheelForward();
    m_rearLeft.WheelForward();
}
