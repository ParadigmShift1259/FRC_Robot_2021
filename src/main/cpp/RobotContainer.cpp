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

#include "commands/DriveForward.h"
#include "commands/FlywheelIdle.h"
#include "commands/FlywheelRamp.h"
#include "commands/HoodRaise.h"
#include "commands/IntakeIngest.h"
#include "commands/IntakeRelease.h"
#include "commands/TurretControl.h"
#include "commands/Shoot.h"

using namespace DriveConstants;
using namespace CyclerConstants;

DriveSubsystem *g_drive = nullptr;

RobotContainer::RobotContainer(Logger& log)
    : m_log(log)
    , m_drive(log)
    //, m_hood()
{
    // Initialize all of your commands and subsystems here

    g_drive = &m_drive;

    // Configure the button bindings
    ConfigureButtonBindings();
    m_fieldRelative = false;

    // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            // up is xbox joystick y pos
            // left is xbox joystick x pos
            auto xInput = Deadzone(m_driverController.GetY(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneX);
            auto yInput = Deadzone(m_driverController.GetX(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneY);
            /*
            if (xInput >= 0)
                xInput = xInput*xInput; // square
            else
                xInput = -xInput*xInput; // square
            if (yInput >= 0)
                yInput = yInput*yInput; // square
            else
                yInput = -yInput*yInput; // square
            */
            auto rotInput = Deadzone(m_driverController.GetX(frc::GenericHID::kRightHand) * -1.0, OIConstants::kDeadzoneRot);
            auto xRot = m_driverController.GetY(frc::GenericHID::kRightHand) * -1.0;
            auto yRot = m_driverController.GetX(frc::GenericHID::kRightHand) * -1.0;
            if (Deadzone(sqrt(pow(xRot, 2) + pow(yRot, 2)), OIConstants::kDeadzoneAbsRot) == 0) {
                xRot = 0;
                yRot = 0;
            }

            m_inputXentry.SetDouble(xInput);
            m_inputYentry.SetDouble(yInput);
            m_inputRotentry.SetDouble(rotInput);

            if (m_fieldRelative)
            {
                m_drive.RotationDrive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed), // TO DO: add teleOp const for max speed
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed), // TO DO: add teleOp const for max speed
                            xRot,
                            yRot,
                            m_fieldRelative);
            }
            else 
            {
                m_drive.Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed), // TO DO: add teleOp const for max speed
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed), // TO DO: add teleOp const for max speed
                            units::radians_per_second_t(rotInput * AutoConstants::kMaxAngularSpeed),
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

    (frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperLeft).WhenHeld(&m_enableFieldRelative));
    (frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperLeft).WhenReleased(&m_disableFieldRelative));

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

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStart).WhenPressed(
        std::move(*(frc2::SequentialCommandGroup*)GetAutonomousCommand())
    );

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperRight).WhenPressed(
            frc2::InstantCommand(    
            [this] {
                m_drive.ZeroHeading();
            },
            {&m_drive}
        )
    );

    /*
    /// Interferes with flywheel
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStickLeft).WhenPressed(
        &m_feedermotor.Set(CyclerConstants::kFeederSpeed)).WithTimeout(CyclerConstants::kFeederTimeout);

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStickRight).WhenPressed(
        &m_feedermotor.Set(CyclerConstants::kFeederSpeed)).WithTimeout(CyclerConstants::kFeederTimeoutAlt);
    // Interferes with flywheel
    */

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBack).WhenPressed(
        HoodRaise(&m_hood)
    );
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStickLeft).WhenPressed(
        FlywheelIdle(&m_flywheel)
    );
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStickRight).WhenPressed(
        FlywheelRamp(&m_flywheel)
    );
}

void RobotContainer::PrintTrajectory(frc::Trajectory& trajectory)
{
    for (auto &state:trajectory.States())
    {
        double time = state.t.to<double>();
        double x = state.pose.X().to<double>();
        double y = state.pose.Y().to<double>();
        printf("%.3f, %.3f, %.3f\n", time, x, y);
    }
}

// frc::Rotation2d RobotContainer::GetDesiredRotation() { return m_drive.GetHeadingAsRot2d(); }

// frc::Rotation2d GetDesiredRotation() { return g_drive->GetHeadingAsRot2d(); }

frc::Rotation2d GetDesiredRotation() { return frc::Rotation2d(0_deg); }

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    //m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

    // Set up config for trajectory
    // frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    frc::TrajectoryConfig config( units::meters_per_second_t (AutoConstants::kMaxSpeed), AutoConstants::kMaxAcceleration);
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
    wpi::sys::path::append(deployDirectory, "paths/output"); //Has the projects that are created in meters
    wpi::sys::path::append(deployDirectory, "AutoNavBarrel.wpilib.json");
    frc::Trajectory exampleTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);    
*/
    // auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(TestTrajLine, config);
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBarrel, config);
    // auto exampleTrajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce1, config);
    // auto exampleTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce2, config);
    // auto exampleTrajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce3, config);
    // auto exampleTrajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce4, config);
    // auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavSlalom, config);

 /*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        TestTrajCircle2,
        config
    );
*/

    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, 0,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-wpi::math::pi), units::radian_t(wpi::math::pi));

    PrintTrajectory(exampleTrajectory); //Comment Out on Bounce Path
    // PrintTrajectory(exampleTrajectory1);
    // PrintTrajectory(exampleTrajectory2);
    // PrintTrajectory(exampleTrajectory3);
    // PrintTrajectory(exampleTrajectory4);


// /* // Comment out on bounce Path
    frc2::SwerveControllerCommand<DriveConstants::kNumSwerveModules> swerveControllerCommand(
        exampleTrajectory,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        //[this]() { return m_drive.GetCurrentModuleStates(); },                  
        frc2::PIDController(AutoConstants::kPXController, 0, 0),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, 0),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        GetDesiredRotation, //Comment out on SwerveControllerCommand1                                                      // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );
// */

/*
    frc2::SwerveControllerCommand<DriveConstants::kNumSwerveModules> getBounceSwerveControllerCommand1(
        exampleTrajectory1,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        //[this]() { return m_drive.GetCurrentModuleStates(); },                  
        frc2::PIDController(AutoConstants::kPXController, 0, 0),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, 0),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        GetDesiredRotation, //Comment out on SwerveControllerCommand1                                                      // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );

    frc2::SwerveControllerCommand<DriveConstants::kNumSwerveModules> getBounceSwerveControllerCommand2(
        exampleTrajectory2,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        //[this]() { return m_drive.GetCurrentModuleStates(); },                  
        frc2::PIDController(AutoConstants::kPXController, 0, 0),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, 0),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        GetDesiredRotation, //Comment out on SwerveControllerCommand1                                                      // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );

    frc2::SwerveControllerCommand<DriveConstants::kNumSwerveModules> getBounceSwerveControllerCommand3(
        exampleTrajectory3,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        //[this]() { return m_drive.GetCurrentModuleStates(); },                  
        frc2::PIDController(AutoConstants::kPXController, 0, 0),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, 0),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        GetDesiredRotation, //Comment out on SwerveControllerCommand1                                                      // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );

    frc2::SwerveControllerCommand<DriveConstants::kNumSwerveModules> getBounceSwerveControllerCommand4(
        exampleTrajectory4,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        //[this]() { return m_drive.GetCurrentModuleStates(); },                  
        frc2::PIDController(AutoConstants::kPXController, 0, 0),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, 0),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        GetDesiredRotation, //Comment out on SwerveControllerCommand1                                                      // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );
*/

    // Reset odometry to the starting pose of the trajectory
    m_drive.ResetOdometry(exampleTrajectory.InitialPose()); //Comment out when running bounce p nath
    // m_drive.ResetOdometry(exampleTrajectory1.InitialPose());

    return new frc2::SequentialCommandGroup(
        std::move(swerveControllerCommand), //Comment out when running bounce path
        // std::move(getBounceSwerveControllerCommand1),
        // std::move(getBounceSwerveControllerCommand2),
        // std::move(getBounceSwerveControllerCommand3),
        // std::move(getBounceSwerveControllerCommand4),
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
