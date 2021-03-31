/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include "commands/DriveToBall.h"
#include "commands/FindClosestBall.h"
#include "commands/RotateToFindNextBall.h"
#include "commands/DriveToFinishLine.h"


// Commenting this out reduces build time by about half
// However, includes are necessary to run trajectory paths
#define PATHS

// Commenting this out removes subsystem CAN errors
// Use this on the Mk2 swerve bot chassis that doesn't have any of the subsystems ready
//#define SUBSYSTEMS
#include "TestTraj.h"
#ifdef PATHS
#include "AutoNavBarrel.h"
#include "AutoNavBounce.h"
#include "AutoNavSlalom.h"
#include "GSLayout1Path1.h"
#include "GSLayout1Path2.h"
#include "GSLayout2Path1.h"
#include "GSLayout2Path2.h"
#endif

using namespace DriveConstants;

RobotContainer::RobotContainer()
    : m_drive()
    , m_flywheel()
    , m_turret()
    , m_hood()
    , m_intake()
    , m_cycler()
    , m_vision()
    // , m_climber()
{
    // Initialize all of your commands and subsystems here
    m_fieldRelative = false;

    m_turretready = false;
    m_firing = false;
    m_finished = false;

    // Configure the button bindings
    ConfigureButtonBindings();
    SetDefaultCommands();
}

void RobotContainer::Periodic()
{
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

    m_turret.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                auto turretXRot = m_secondaryController.GetY(frc::GenericHID::kRightHand) * -1.0;
                auto turretYRot = m_secondaryController.GetX(frc::GenericHID::kRightHand);
                if (Deadzone(sqrt(pow(turretXRot, 2) + pow(turretYRot, 2)), OIConstants::kDeadzoneAbsRot) == 0) {
                    turretXRot = 0;
                    turretYRot = 0;
                }
                double rotPosition = atan2f(turretYRot, turretXRot);
                rotPosition *= 360.0/Math::kTau; 
                m_turret.TurnToRobot(rotPosition);
            }, {&m_turret}
        )
    );

    m_hood.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                m_hood.Set(HoodConstants::kMax);
            }, {&m_hood}
        )
    );
    
    // m_climber.SetDefaultCommand(
    //     frc2::RunCommand(
    //         [this] {
    //             m_climber.Run(0);
    //         }, {&m_climber}
    //     )
    // );

    m_intake.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                m_intake.Set(0);
            }, {&m_intake}
        )
    );

    m_cycler.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                m_cycler.SetFeeder(0);
                m_cycler.SetTurnTable(0);
            }, {&m_cycler}
        )
    );
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
    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kBumperLeft).WhenPressed(
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

    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kY).WhenPressed(
        std::move(*(frc2::SequentialCommandGroup*)GetDriveTestCommand(kFront))
    );

    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kX).WhenPressed(
        std::move(*(frc2::SequentialCommandGroup*)GetDriveTestCommand(kLeft))
    );

    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kB).WhenPressed(
        std::move(*(frc2::SequentialCommandGroup*)GetDriveTestCommand(kRight))
    );

    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kA).WhenPressed(
        std::move(*(frc2::SequentialCommandGroup*)GetDriveTestCommand(kBack))
    );

    // frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kA).WhenPressed(
    //     frc2::InstantCommand(    
    //     [this] {
    //         m_drive.ResetRelativeToAbsolute();
    //     },
    //     {&m_drive}
    //     )
    // );

    // Triggers Fire sequence
    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kY).WhenPressed(
        Fire(&m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision,
             &m_turretready, &m_firing, &m_finished)
    );

    // Runs sequence of tests for motors based on iterator and a power
    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kA).WhenHeld(
        CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed)   
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kBumperLeft).WhenPressed(
        CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeedHigher)   
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kA).WhenReleased(
        CyclerPrepare(&m_cycler, true).WithTimeout(CyclerConstants::kMaxCyclerTime)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kBack).WhenHeld(
        Unjam(&m_cycler, &m_intake)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kB).WhenHeld(
        IntakeRelease(&m_intake)
    );

    // frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kBumperLeft).WhenPressed(
    //     frc2::InstantCommand(    
    //     [this] {
    //         m_turret.SetNewPIDValues();
    //     },
    //     {&m_turret}
    //     )
    // );
}

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
    //auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBarrel, config);
    // auto exampleTrajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce1, config);
    // auto exampleTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce2, config);
    // auto exampleTrajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce3, config);
    // auto exampleTrajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce4, config);
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavSlalom, config);

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

    //PrintTrajectory(exampleTrajectory); //Comment Out on Bounce Path
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

frc2::Command *RobotContainer::GetDriveTestCommand(Direction direction)
{

    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);
    frc::Trajectory exampleTrajectory;

    m_drive.ZeroHeading();
    // Reset odometry to the starting pose of the trajectory
    m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

    switch (direction)
    {
        case kFront:
            exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                TestTrajFront,
                config
            );
            break;
        case kLeft:
            exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                TestTrajLeft,
                config
            );
            break;
        case kRight:
            exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                TestTrajRight,
                config
            );
            break;
        case kBack:
            exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                TestTrajBack,
                config
            );
            break;
    }

    frc::ProfiledPIDController<units::radians> thetaController {
        AutoConstants::kPThetaController, 0, 0,
        AutoConstants::kThetaControllerConstraints
    };

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