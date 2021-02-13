/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <iostream>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/CyclerSubsystem.h"
#include "SwerveControllerCommand2.h"

#include "AutoNavBarrel.h"
#include "AutoNavBounce.h"
#include "AutoNavSlalom.h"
#include "GSLayout1Path1.h"
#include "GSLayout1Path2.h"
#include "GSLayout2Path1.h"
#include "GSLayout2Path2.h"
#include "TestTraj.h"

using namespace DriveConstants;
using namespace CyclerConstants;

RobotContainer::RobotContainer(Logger& log)
    : m_log(log)
    , m_drive(log)
    , m_flywheel()
    , m_turret()
    , m_hood()
    , m_intake()
    , m_cycler()
    , m_vision()
{
    // Initialize all of your commands and subsystems here
    m_fieldRelative = false;

    // Configure the button bindings
    ConfigureButtonBindings();

    // Set up default commands
    m_drive.SetDefaultCommand(
        DriveDefault(&m_drive, 
            [this] {
                double x = Deadzone(m_driverController.GetY(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneX);
                m_inputXentry.SetDouble(x);
                return x;
            },
            [this] {
                double y = Deadzone(m_driverController.GetX(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneY);
                m_inputYentry.SetDouble(y);
                return y;
            },
            [this] {
                double rot = Deadzone(m_driverController.GetX(frc::GenericHID::kRightHand) * -1.0, OIConstants::kDeadzoneRot);
                m_inputRotentry.SetDouble(rot);
                return rot;
            },
            [this] {
                return m_driverController.GetY(frc::GenericHID::kRightHand) * -1.0;
            }, 
            [this] {
                return m_driverController.GetX(frc::GenericHID::kRightHand) * -1.0;
            },
            [this] {
                return m_fieldRelative;
            }
        )
    );

    ShuffleboardTab& tab = Shuffleboard::GetTab("XboxInput");
    m_inputXentry = tab.Add("X", 0).GetEntry();
    m_inputYentry = tab.Add("Y", 0).GetEntry();
    m_inputRotentry = tab.Add("Rot", 0).GetEntry();

    m_testNumber = 0;
    m_testPower = 0;

    SmartDashboard::PutNumber("TEST_testNumber", m_testNumber);
    SmartDashboard::PutNumber("TEST_testPower", m_testPower);
}

void RobotContainer::Periodic()
{
    m_testNumber = (int) SmartDashboard::GetNumber("TEST_testNumber", 0);
    m_testPower = SmartDashboard::GetNumber("TEST_testPower", 0);
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

    // Triggers Fire sequence
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY).WhenHeld(
        Fire(&m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision)
    );

    // Triggers field relative driving
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperLeft).WhenHeld(
        frc2::InstantCommand(    
            [this] { m_fieldRelative = true; },
            {}
        )
    );
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperLeft).WhenReleased(
        frc2::InstantCommand(    
            [this] { m_fieldRelative = false; },
            {}
        )
    );

    // Zeros the gyro
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperRight).WhenPressed(
        frc2::InstantCommand(    
        [this] {
            m_drive.ZeroHeading();
        },
        {&m_drive}
        )
    );

    // Runs autonomous path in gyro
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStart).WhenPressed(
        std::move(*(frc2::SequentialCommandGroup*)GetAutonomousCommand())
    );

    // Runs sequence of tests for motors based on iterator and a power
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA).WhenHeld(
        TestCommands()
    );
    /*

    double c_buttonInputSpeed = 0.5;
    units::second_t c_buttonInputTime = 1.25_s;

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY).WhenPressed(
        frc2::RunCommand(    
            [this, c_buttonInputSpeed] {
                m_drive.Drive(units::meters_per_second_t(c_buttonInputSpeed),
                        units::meters_per_second_t(0),
                        units::radians_per_second_t(0),
                        false);
            },
            {&m_drive}
        ).WithTimeout(c_buttonInputTime));

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA).WhenPressed(
            frc2::RunCommand(    
            [this, c_buttonInputSpeed] {
                m_drive.Drive(units::meters_per_second_t(-c_buttonInputSpeed),
                        units::meters_per_second_t(0),
                        units::radians_per_second_t(0),
                        false);
            },
            {&m_drive}
        ).WithTimeout(c_buttonInputTime));

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kX).WhenPressed(
            frc2::RunCommand(    
            [this, c_buttonInputSpeed] {
                m_drive.Drive(units::meters_per_second_t(0),
                        units::meters_per_second_t(c_buttonInputSpeed),
                        units::radians_per_second_t(0),
                        false);
            },
            {&m_drive}
        ).WithTimeout(c_buttonInputTime));

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kB).WhenPressed(
            frc2::RunCommand(    
            [this, c_buttonInputSpeed] {
                m_drive.Drive(units::meters_per_second_t(0),
                        units::meters_per_second_t(-c_buttonInputSpeed),
                        units::radians_per_second_t(0),
                        false);
            },
            {&m_drive}
        ).WithTimeout(c_buttonInputTime));

    */
}

frc2::InstantCommand RobotContainer::TestCommands()
{
    switch(m_testNumber) {
    case 0:
        return 
        frc2::InstantCommand(    
            [this] {
                m_intake.Set(m_testPower);
            },
            {&m_intake}
        );
    case 1:
        return
        frc2::InstantCommand(    
            [this] {
                m_cycler.SetFeeder(m_testPower);
            },
            {&m_cycler}
        );
    case 2:
        return 
        frc2::InstantCommand(    
            [this] {
                m_cycler.SetTurnTable(m_testPower);
            },
            {&m_cycler}
        );
    case 3:
        return 
        frc2::InstantCommand(    
            [this] {
                m_flywheel.SetRPM(m_testPower * 1000.0);
            },
            {&m_flywheel}
        );
        break;
    case 4:
        return 
        frc2::InstantCommand(    
            [this] {
                m_hood.Set(m_testPower);
            },
            {&m_hood}
        );
    }
    return 
        frc2::InstantCommand(    
            [this] {
                m_intake.Set(m_testPower);
            },
            {&m_intake}
        );
}

// frc::Rotation2d RobotContainer::GetDesiredRotation() { return m_drive.GetHeadingAsRot2d(); }

frc::Rotation2d GetDesiredRotation() { return frc::Rotation2d(0_deg); }

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);

/*
    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, 1_m)},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(3_m, 1_m, frc::Rotation2d(0_deg)),
        // Pass the config
        config
    );
*/

/*
    wpi::SmallString<64> deployDirectory;
    frc::filesystem::GetDeployDirectory(deployDirectory);
    wpi::sys::path::append(deployDirectory, "paths");
    wpi::sys::path::append(deployDirectory, "AutoNavBarrel.wpilib.json");

    frc::Trajectory exampleTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
*/

// /*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        TestTrajCircle2,
        config
    );
    std::cout << "Number of Trajectory States: \n" << exampleTrajectory.States().size();
// */

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        AutoNavBarrel,
        config
    );
*/

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        AutoNavBounce,
        config
    );
*/

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        AutoNavSlalom,
        config
    );
*/

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        GSLayout1Path1,
        config
    );
*/

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        GSLayout1Path2,
        config
    );
*/

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        GSLayout2Path1,
        config
    );
*/

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        GSLayout2Path2,
        config
    );
*/

    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, 0,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-wpi::math::pi), units::radian_t(wpi::math::pi));

    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> swerveControllerCommand(
        exampleTrajectory,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(AutoConstants::kPXController, 0, 0),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, 0),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        // GetDesiredRotation,                                                     // std::function< frc::Rotation2d()> desiredRotation
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
