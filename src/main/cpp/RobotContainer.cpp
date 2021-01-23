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

#include "commands/DriveForward.h"

using namespace DriveConstants;

RobotContainer::RobotContainer(Logger& log)
    : m_log(log)
    , m_drive(log)
{
    // Initialize all of your commands and subsystems here

    //m_driveForward = m_driveForward.WithTimeout(2.0_s);
    /*
    m_driveForward = new frc2::RunCommand(
        [this, c_buttonInputSpeed] () {
            m_drive.Drive(units::meters_per_second_t(c_buttonInputSpeed),
                            units::meters_per_second_t(0),
                            units::radians_per_second_t(0),
                            m_fieldRelative);
        },
        {&m_drive}
    );*/

    // Configure the button bindings
    ConfigureButtonBindings();
    m_fieldRelative = false;

    // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {

//#define USE_BUTTONS
#ifdef USE_BUTTONS
            // Push button control to diagnose swerve angle accuracy
            double xInput = 0.0;
            const double c_buttonInputSpeed = 0.5;
            if (m_driverController.GetYButton())
            {
                xInput = c_buttonInputSpeed;
            }
            else if (m_driverController.GetAButton())
            {
                xInput = -1.0 * c_buttonInputSpeed;
            }
            double yInput = 0.0;
            if (m_driverController.GetXButton())
            {
                yInput = c_buttonInputSpeed;
            }
            else if (m_driverController.GetBButton())
            {
                yInput = -1.0 * c_buttonInputSpeed;
            }

            double rotInput = 0.0;
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
            auto dPadPointOfView = m_driverController.GetPOV();
            if (dPadPointOfView >= 225 && dPadPointOfView <= 315)
            {
                rotInput = 1.0;
            }
            else if (dPadPointOfView >= 45 && dPadPointOfView <= 135)
            {
                rotInput = -1.0;
            }
#else
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

#endif

            m_inputXentry.SetDouble(xInput);
            m_inputYentry.SetDouble(yInput);
            m_inputRotentry.SetDouble(rotInput);

            /// \todo Scale +/-1.0 xbox input to kMaxSpeed
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

    // The roboRIO does not have a battery powered RTC. However, the DS sends the time when it connects, which the roboRIO uses to set the system time. If you wait until the DS connects, you can have correct timestamps, without a RTC.
    double matchTime = frc::Timer::GetMatchTime();
    printf("Match time %.3f\n", matchTime);

    // Shuffleboard::GetTab("Preround").Add("Partner can scale", false)
    //                                 .WithWidget(frc::BuiltInWidgets::kSplitButtonChooser)
    //                                 .WithSize(2, 1)     // Widget size on shuffleboard
    //                                 .WithPosition(0,0); // Widget position on shuffleboard
}

void RobotContainer::ConfigureButtonBindings()
{
    // Configure your button bindings here
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
}

frc::Rotation2d GetDesiredRotation() { return frc::Rotation2d(0_deg); }

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);

//*
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
//*/

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {frc::Translation2d(1_m, 0_m), frc::Translation2d(2_m, 0_m)},
        // End 2 meters straight ahead of where we started, facing forward
        frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass the config
        config
    );
*/

    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, 0,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-wpi::math::pi), units::radian_t(wpi::math::pi));

    //printf("SwerveControllerCommand exampleTrajectory.States().size() %u\n", exampleTrajectory.States().size());

    //auto rot = exampleTrajectory.States().back().pose.Rotation();
    //printf("SwerveControllerCommand exampleTrajectory.States().back().pose.Rotation() %.3f\n", rot.Degrees().to<double>());

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

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());

    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(swerveControllerCommand),
        //std::move(swerveControllerCommand),
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
