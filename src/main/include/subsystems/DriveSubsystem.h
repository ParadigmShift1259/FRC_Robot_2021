/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix.h>

#include "Constants.h"
#include "SwerveModule.h"
#include "Logger.h"

// For each enum here, add a string to c_headerNamesDriveSubsystem
// and a line like this: 
//      m_logData[EDriveSubSystemLogData::e???] = ???;
// to DriveSubsystem::Periodic
enum class EDriveSubSystemLogData : int
{
    eFirstInt
  , eLastInt = eFirstInt

  , eFirstDouble
  , eInputX = eFirstDouble
  , eInputY
  , eInputRot
  , eOdoX
  , eOdoY
  , eOdoRot
  , eLastDouble
  , eGyroRot
  , eGyroRotRate
};

const std::vector<std::string> c_headerNamesDriveSubsystem{
       "InputX"
     , "InputY"
     , "InputRot"
     , "OdoX"
     , "OdoY"
     , "OdoRot"
     , "eGyroRot"
     , "eGyroRotRate"
};

class DriveSubsystem : public frc2::SubsystemBase
{
public:
    enum EModuleLocation    //!< Order as returned by kDriveKinematics.ToSwerveModuleStates
    {
        eFrontLeft,
        eFrontRight,
        eRearLeft,
        eRearRight
    };

    DriveSubsystem(Logger& log);

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    // Subsystem methods go here.

    /// Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
    /// and the linear speeds have no effect on the angular speed.
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param rot           Angular rate of the robot.
    /// \param fieldRelative Whether the provided x and y speeds are relative to the field.
    void Drive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rot, bool fieldRelative);

    // Drives the robot with the right stick controlling the position angle of the robot
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param xRot          Angle of the robot on the x axis
    /// \param yRot          Angle of the robot on the y axis
    /// \param fieldRelative Whether the provided translational speeds are relative to the field.
    void RotationDrive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, double xRot, double yRot, bool fieldRelative);

    /// Resets the drive encoders to currently read a position of 0.
    void ResetEncoders();

    /// Readable alias for array of swerve modules
    using SwerveModuleStates = wpi::array<frc::SwerveModuleState, DriveConstants::kNumSwerveModules>;
    /// Sets the drive SpeedControllers to a power from -1 to 1.
    void SetModuleStates(SwerveModuleStates desiredStates);

    /// Returns the heading of the robot.
    /// \return the robot's heading in degrees, from -180 to 180
    double GetHeading();
    frc::Rotation2d GetHeadingAsRot2d() { return frc::Rotation2d(degree_t(GetHeading())); }

    /// Zeroes the heading of the robot.
    void ZeroHeading();

    /// Returns the turn rate of the robot.
    /// \return The turn rate of the robot, in degrees per second
    double GetTurnRate();

    /// Returns the currently-estimated pose of the robot.
    /// \return The pose.
    frc::Pose2d GetPose();

    /// Resets the odometry to the specified pose.
    /// \param pose The pose to which to set the odometry.
    void ResetOdometry(frc::Pose2d pose);

    /// The log header flag must be reset everyt time the robot is enabled
    void ResetLog()
    { 
        m_logData.ResetHeaderLogged();
        m_frontLeft.ResetLog();
        m_frontRight.ResetLog();
        m_rearRight.ResetLog();
        m_rearLeft.ResetLog();
    }

    meter_t kTrackWidth = 21.5_in; //!< Distance between centers of right and left wheels on robot
    meter_t kWheelBase = 23.5_in;  //!< Distance between centers of front and back wheels on robot

    /// The kinematics object converts inputs into 4 individual swerve module turn angle and wheel speeds
    frc::SwerveDriveKinematics<DriveConstants::kNumSwerveModules> kDriveKinematics{
        frc::Translation2d( kWheelBase / 2,  kTrackWidth / 2),    // +x, +y FL
        frc::Translation2d( kWheelBase / 2, -kTrackWidth / 2),    // +x, -y FR
        frc::Translation2d(-kWheelBase / 2,  kTrackWidth / 2),    // -x, +y RL
        frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2)};   // -x, -y RR

private:    
    /// Get all 4 swerve module wheel speed to update the odometry with
    SwerveModuleStates getCurrentWheelSpeeds()
    {
        SwerveModuleStates sms(
            m_frontLeft.GetState(),
            m_frontRight.GetState(),
            m_rearRight.GetState(),
            m_rearLeft.GetState()
        );
        return sms;
    }

    /// \name Logging helpers
    /// Log important data to a file on the Roborio
    ///@{
    using LogData = LogDataT<EDriveSubSystemLogData>;
    Logger& m_log;
    LogData m_logData;
    ///@}

    /// \name Swerve Modules
    /// The drive subsystem owns all 4 swerve modules
    ///@{
    SwerveModule m_frontLeft;
    SwerveModule m_frontRight;
    SwerveModule m_rearRight;
    SwerveModule m_rearLeft;
    ///@}

    PigeonIMU m_gyro;                                       //!< Inertial measurement unit; compass + accelerometer

    /// Odometry class for tracking robot pose
    frc::SwerveDriveOdometry<DriveConstants::kNumSwerveModules> m_odometry;

    /// PID to control overall robot chassis rotation 
    frc2::PIDController m_rotationPIDController{
        DriveConstants::kRotationP,
        DriveConstants::kRotationI,
        DriveConstants::kRotationD
    };
};
