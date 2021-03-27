/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"


// Commenting this out reduces build time by about half
// However, includes are necessary to run trajectory paths
//#define PATHS

// Commenting this out removes subsystem CAN errors
// Use this on the Mk2 swerve bot chassis that doesn't have any of the subsystems ready
//#define SUBSYSTEMS

#ifdef PATHS
#include "AutoNavBarrel.h"
#include "AutoNavBounce.h"
#include "AutoNavSlalom.h"
#include "GSLayout1Path1.h"
#include "GSLayout1Path2.h"
#include "GSLayout2Path1.h"
#include "GSLayout2Path2.h"
#include "TestTraj.h"
#endif

using namespace DriveConstants;

RobotContainer::RobotContainer(Logger& log, const int& lowPrioritySkipCount)
    : m_log(log)
    , m_drive(log, lowPrioritySkipCount)
    , m_lowPrioritySkipCount(lowPrioritySkipCount)
{
    // Initialize all of your commands and subsystems here
    m_fieldRelative = false;

    // Configure the button bindings
    ConfigureButtonBindings();
    SetDefaultCommands();

    m_testNumber = 0;
    m_testPower = 0;

    SmartDashboard::PutNumber("TEST_R_number", m_testNumber);
    SmartDashboard::PutNumber("TEST_R_power", m_testPower);
}

void RobotContainer::Periodic()
{
    if (m_lowPrioritySkipCount % 5 == 0)    // 10 per second
    {
        m_testNumber = (int) SmartDashboard::GetNumber("TEST_testNumber", 0);
        m_testPower = SmartDashboard::GetNumber("TEST_testPower", 0);
    }
}

void RobotContainer::SetDefaultCommands()
{
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            // up is xbox joystick y pos
            // left is xbox joystick x pos
            auto xInput = Deadzone(m_primaryController.GetY(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneX);
            auto yInput = Deadzone(m_primaryController.GetX(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneY);
            auto rotInput = Deadzone(m_primaryController.GetX(frc::GenericHID::kRightHand) * -1.0, OIConstants::kDeadzoneRot);
            auto xRot = m_primaryController.GetY(frc::GenericHID::kRightHand) * -1.0;
            auto yRot = m_primaryController.GetX(frc::GenericHID::kRightHand) * -1.0;
            if (Deadzone(sqrt(pow(xRot, 2) + pow(yRot, 2)), OIConstants::kDeadzoneAbsRot) == 0) {
                xRot = 0;
                yRot = 0;
            }

            m_inputXentry.SetDouble(xInput);
            m_inputYentry.SetDouble(yInput);
            m_inputRotentry.SetDouble(rotInput);

            if (m_fieldRelative)
            {
                m_drive.RotationDrive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                            xRot,
                            yRot,
                            m_fieldRelative);
            }
            else 
            {
                m_drive.HeadingDrive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                            units::radians_per_second_t(rotInput),
                            m_fieldRelative);
            }

        },
        {&m_drive}
    ));

    ShuffleboardTab& tab = Shuffleboard::GetTab("XboxInput");
    m_inputXentry = tab.Add("X", 0).GetEntry();
    m_inputYentry = tab.Add("Y", 0).GetEntry();
    m_inputRotentry = tab.Add("Rot", 0).GetEntry();
}


void RobotContainer::ConfigureButtonBindings()
{
    //            U           //
    //            0           //
    //            |           //
    //    UL 315\ | /45 UR    //
    //           \|/          //
    // L 270------+------90 R //
    //           /|\          //
    //    DL 225/ | \135 DR   //
    //            |           //
    //           180          //
    //            D           //

    // Triggers field relative driving
    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kBumperLeft).WhenHeld(
        frc2::InstantCommand(    
            [this] { m_fieldRelative = true; },
            {}
        )
    );
    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kBumperLeft).WhenReleased(
        frc2::InstantCommand(    
            [this] { m_fieldRelative = false; },
            {}
        )
    );

    // Zeros the gyro
    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kBumperRight).WhenPressed(
        frc2::InstantCommand(    
        [this] {
            m_drive.ZeroHeading();
        },
        {&m_drive}
        )
    );

    // Runs autonomous path in gyro
    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kStart).WhenPressed(
        std::move(*(frc2::SequentialCommandGroup*)GetAutonomousCommand())
    );

    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kA).WhenPressed(
        frc2::InstantCommand(    
        [this] {
            m_drive.ResetRelativeToAbsolute();
        },
        {&m_drive}
        )
    );
}


// frc::Rotation2d RobotContainer::GetDesiredRotation() { return m_drive.GetHeadingAsRot2d(); }

frc::Rotation2d GetDesiredRotation() { return frc::Rotation2d(0_deg); }

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    //m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);

    wpi::SmallString<64> deployDirectory;
    frc::filesystem::GetDeployDirectory(deployDirectory);
    wpi::sys::path::append(deployDirectory, "paths/output"); //Has the projects that are created in meters
    wpi::sys::path::append(deployDirectory, "AutoNavBarrel.wpilib.json");

    frc::Trajectory exampleTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, 0,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-wpi::math::pi), units::radian_t(wpi::math::pi));

    frc2::SwerveControllerCommand<DriveConstants::kNumSwerveModules> swerveControllerCommand(
        exampleTrajectory,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(AutoConstants::kPXController, 0, 0),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, 0),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        GetDesiredRotation,                                                     // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );

    // Reset odometry to the starting pose of the trajectory
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());

    return new frc2::SequentialCommandGroup(
        std::move(swerveControllerCommand),
        frc2::InstantCommand(
            [this]() {
                m_drive.Drive(units::meters_per_second_t(0.0),
                              units::meters_per_second_t(0.0),
                              units::radians_per_second_t(0.0), false);
            },
            {}
        )
    );
}


frc2::Command *RobotContainer::GetAutonomousGSCommand()
{

    return new frc2::SequentialCommandGroup(
        frc2::InstantCommand(
            [this]() {
                m_drive.Drive(units::meters_per_second_t(0.0),
                              units::meters_per_second_t(0.0),
                              units::radians_per_second_t(0.0), false);
            },
            {}
        )
    );
}