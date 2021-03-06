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
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "AutoNavSlalom.h"

#include "AutoNavBarrel.h"
#include "AutoNavBounce.h"
/*
#include "GSLayout1Path1.h"
#include "GSLayout1Path2.h"
#include "GSLayout2Path1.h"
#include "GSLayout2Path2.h"
*/

#include "commands/DriveForward.h"

using namespace DriveConstants;

RobotContainer::RobotContainer(Logger& log)
    : m_log(log)
    , m_drive(log)
    , m_intake()
{
    // Initialize all of your commands and subsystems here
    m_fieldRelative = false;

    // Configure the button bindings
    ConfigureButtonBindings();
    //SetDefaultCommands();

    m_testNumber = 0;
    m_testPower = 0.75;
    SmartDashboard::PutNumber("TEST_testNumber", m_testNumber);
    SmartDashboard::PutNumber("TEST_testPower", m_testPower);

    // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            // up is xbox joystick y pos
            // left is xbox joystick x pos
            auto xInput = Deadzone(m_driverController.GetY(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneX);
            auto yInput = Deadzone(m_driverController.GetX(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneY);
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
                m_drive.RotationDrive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                            xRot,
                            yRot,
                            m_fieldRelative);
            }
            else 
            {
                m_drive.Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
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

void RobotContainer::Periodic()
{
    m_testNumber = (int) SmartDashboard::GetNumber("TEST_testNumber", 0);
    m_testPower = SmartDashboard::GetNumber("TEST_testPower", 0);
    m_drive.Periodic();
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
    units::second_t c_buttonInputTime = 5.0_s;// 1.25_s;

    // Increments / Decrements a test power value for TestCommands()
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY).WhenPressed(
        frc2::InstantCommand(    
        [this] {
            m_testPower += 0.05;
            SmartDashboard::PutNumber("TEST_testPower", m_testPower);
            printf("Y button ++ %.3f\n", m_testPower);
        },
        {}
        )
    );

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kX).WhenPressed(
        frc2::InstantCommand(    
        [this] {
            m_testPower -= 0.05;
            SmartDashboard::PutNumber("TEST_testPower", m_testPower);
            printf("X button -- %.3f\n", m_testPower);
        },
        {}
        )
    );

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kB).WhenHeld(
        frc2::InstantCommand(    
        [this] {
            m_testPower = 0.0;
            SmartDashboard::PutNumber("TEST_testPower", m_testPower);
            printf("B button -- %.3f\n", m_testPower);
            TestCommands();
        },
        {}
        )
    );

    // Runs sequence of tests for motors based on iterator and a power
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA).WhenHeld(
        TestCommands()  // No semi colon
    );

    // frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY).WhenPressed(
    //     frc2::RunCommand(    
    //         [this, c_buttonInputSpeed] {
    //             m_drive.Drive(units::meters_per_second_t(c_buttonInputSpeed),
    //                     units::meters_per_second_t(0),
    //                     units::radians_per_second_t(0),
    //                     false);
    //         },
    //         {&m_drive}
    //     ).WithTimeout(c_buttonInputTime));
    
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStickRight).WhenHeld(
        frc2::RunCommand(    
            [this, c_buttonInputSpeed] {
                m_drive.Drive(units::meters_per_second_t(c_buttonInputSpeed),
                        units::meters_per_second_t(0),
                        units::radians_per_second_t(0),
                        false);
            },
            {&m_drive}
        ));

    // frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA).WhenPressed(
    //         frc2::RunCommand(    
    //         [this, c_buttonInputSpeed] {
    //             m_drive.Drive(units::meters_per_second_t(-c_buttonInputSpeed),
    //                     units::meters_per_second_t(0),
    //                     units::radians_per_second_t(0),
    //                     false);
    //         },
    //         {&m_drive}
    //     ).WithTimeout(c_buttonInputTime));

    // frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA).WhenPressed(
    //         frc2::RunCommand(    
    //         [this] {
    //             m_drive.TemporaryRunTurnMotor();
    //         },
    //         {&m_drive}
    //     ).WithTimeout(c_buttonInputTime));

    // frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kX).WhenPressed(
    //         frc2::RunCommand(    
    //         [this, c_buttonInputSpeed] {
    //             m_drive.Drive(units::meters_per_second_t(0),
    //                     units::meters_per_second_t(c_buttonInputSpeed),
    //                     units::radians_per_second_t(0),
    //                     false);
    //         },
    //         {&m_drive}
    //     ).WithTimeout(c_buttonInputTime));

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
}

frc2::InstantCommand RobotContainer::TestCommands()
{
    // switch(m_testNumber) {
    // case 0:
        return 
        frc2::InstantCommand(    
            [this] {
                printf("m_intake.Set(%.3f)\n", m_testPower);
                m_intake.Set(m_testPower);
            },
            {&m_intake}
        );
    // case 1:
    //     return
    //     frc2::InstantCommand(    
    //         [this] {
    //             m_cycler.SetFeeder(m_testPower);
    //         },
    //         {&m_cycler}
    //     );
    // case 2:
    //     return 
    //     frc2::InstantCommand(    
    //         [this] {
    //             m_cycler.SetTurnTable(m_testPower);
    //         },
    //         {&m_cycler}
    //     );
    // case 3:
    //     return 
    //     frc2::InstantCommand(    
    //         [this] {
    //             m_flywheel.SetRPM(m_testPower * 1000.0);
    //         },
    //         {&m_flywheel}
    //     );
    //     break;
    // case 4:
    //     return 
    //     frc2::InstantCommand(    
    //         [this] {
    //             m_hood.Set(m_testPower);
    //         },
    //         {&m_hood}
    //     );
    //     break;
    // case 5:
    //     return
    //     frc2::InstantCommand(    
    //         [this] {
    //             m_climber.Run(m_testPower);
    //         },
    //         {&m_climber}
    //     );
    // }
    // return 
    //     frc2::InstantCommand(    
    //         [this] {
    //             m_intake.Set(m_testPower);
    //         },
    //         {&m_intake}
    //     );
}

frc::Rotation2d GetDesiredRotation() { return frc::Rotation2d(0_deg); }

frc2::Command *RobotContainer::GetAutonomousCommand()
{
//#define NO_TRAJ
#ifdef NO_TRAJ
    return nullptr;
#else
    m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);

    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBarrel, config);

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
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(1.143_m, 320.583_m, frc::Rotation2d(0_deg)),
        AutoNavSlalom,
        frc::Pose2d(1.524_m, 320.583_m, frc::Rotation2d(0_deg)),
        config
    );
*/

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
#endif
}
