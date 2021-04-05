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
#include <frc2/command/button/NetworkButton.h>

// Commenting this out reduces build time by about half
// However, includes are necessary to run trajectory paths
//#define PATHS

// Commenting this out removes subsystem CAN errors
// Use this on the Mk2 swerve bot chassis that doesn't have any of the subsystems ready
//#define SUBSYSTEMS
#include "TestTraj.h"
#include "AutoCircleTest.h"
#include "AutoNavBarrel.h"
#include "BallSearch.h"
#include "AutoNavSlalom.h"
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

RobotContainer::RobotContainer()
    : m_gyro()
    , m_drive(&m_gyro)
    , m_flywheel()
    , m_turret(&m_gyro)
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
    SmartDashboard::PutBoolean("WheelsForward", false);
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
            auto xInput = pow(Deadzone(m_primaryController.GetY(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneX), 3.0);
            auto yInput = pow(Deadzone(m_primaryController.GetX(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneY), 3.0);
            auto rotInput = pow(Deadzone(m_primaryController.GetX(frc::GenericHID::kRightHand) * -1.0, OIConstants::kDeadzoneRot), 3.0);
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
                            //units::radians_per_second_t(rotInput),
                            xRot,
                            yRot,
                            m_fieldRelative);
            }
            else 
            {
                m_drive.HeadingDrive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                            units::radians_per_second_t(rotInput),
                            // xRot,
                            // yRot,
                            m_fieldRelative);
            }

        },
        {&m_drive}
    ));

    m_turret.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                /* Temporary while running auto testing
                auto turretXRot = m_secondaryController.GetY(frc::GenericHID::kRightHand) * -1.0;
                auto turretYRot = m_secondaryController.GetX(frc::GenericHID::kRightHand);
                if (Deadzone(sqrt(pow(turretXRot, 2) + pow(turretYRot, 2)), OIConstants::kDeadzoneAbsRot) == 0) {
                    turretXRot = 0;
                    turretYRot = 0;
                }
                if (turretXRot == 0 && turretYRot == 0)
                {
                    m_turret.TurnToField(0);
                }
                else {
                    double rotPosition = atan2f(turretYRot, turretXRot);
                    rotPosition *= 360.0/Math::kTau; 
                    m_turret.TurnToRobot(rotPosition);
                }
                */
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
                m_intake.Set(0.7);
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
            m_gyro.ZeroHeading();
        },
        {}
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

    frc2::NetworkButton("SmartDashboard", "WheelsForward").WhenPressed(
        frc2::InstantCommand([this] { m_drive.WheelsForward(); }, { &m_drive} )        
    );
}

frc::Rotation2d GetDesiredRotation() { return frc::Rotation2d(0_deg); }

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // // Set up config for trajectory
    // // frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    // frc::TrajectoryConfig config( units::meters_per_second_t (0.9 * AutoConstants::kMaxSpeed), 0.9 * AutoConstants::kMaxAcceleration);
    // // Add kinematics to ensure max speed is actually obeyed
    // config.SetKinematics(m_drive.kDriveKinematics);

    frc::Trajectory exampleTrajectory = convertArrayToTrajectory(Slalom, sizeof Slalom / sizeof Slalom[0]);
    // auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(TestTrajCircle2, config);
    //auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBarrel, config);
    // auto exampleTrajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce1, config);
    // auto exampleTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce2, config);
    // auto exampleTrajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce3, config);
    // auto exampleTrajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavBounce4, config);
    //auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(AutoNavSlalom, config);
    
    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, AutoConstants::kDThetaController,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-wpi::math::pi), units::radian_t(wpi::math::pi));

    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> swerveControllerCommand(
        exampleTrajectory,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(AutoConstants::kPXController, 0, AutoConstants::kDXController),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, AutoConstants::kDYController),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        //GetDesiredRotation,                                                     // std::function< frc::Rotation2d()> desiredRotation
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
    frc::Trajectory trajectory;
    bool bluePath = false;
    bool finished = false;

    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, AutoConstants::kDThetaController,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-wpi::math::pi), units::radian_t(wpi::math::pi));
    //thetaController.EnableContinuousInput(units::radian_t(-AutoConstants::kMaxAngularSpeed), units::radian_t(AutoConstants::kMaxAngularSpeed));

    trajectory = DetectPath(bluePath, finished);

    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> swerveControllerCommand(
        trajectory,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(AutoConstants::kPXController, 0, AutoConstants::kDXController),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, AutoConstants::kDYController),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        //GetDesiredRotation,                                                     // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );

    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules>* swerveControllerCommandA( 
        convertArrayToTrajectory(BlueA, sizeof BlueA / sizeof BlueA[0]),                                                     // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(AutoConstants::kPXController, 0, AutoConstants::kDXController),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, AutoConstants::kDYController),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        //GetDesiredRotation,                                                     // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );
    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules>* swerveControllerCommandB( 
        convertArrayToTrajectory(BlueB, sizeof BlueB / sizeof BlueB[0]),                                                   // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(AutoConstants::kPXController, 0, AutoConstants::kDXController),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, AutoConstants::kDYController),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        //GetDesiredRotation,                                                     // std::function< frc::Rotation2d()> desiredRotation
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );
    //frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> swerveControllerCommand2;

    // Reset odometry to the starting pose of the trajectory
    m_drive.ResetOdometry(trajectory.InitialPose());

    return new frc2::SequentialCommandGroup(
        std::move(*swerveControllerCommand),
        frc2::InstantCommand(
            [this, bluePath, finished, swerveControllerCommand2, thetaController]() {
                bool bluePath2 = bluePath;
                bool finished2 = finished;
                frc::Trajectory trajectory = DetectPath(bluePath2, finished2);
                m_drive.ResetOdometry(trajectory.InitialPose());
            },
            {}
        ),
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

frc::Trajectory RobotContainer::DetectPath(bool& bluePath, bool& finished)
{
    double angle = SmartDashboard::GetNumber("XAngle", 0);
    double distance = SmartDashboard::GetNumber("ZDistance", -1.0);
    PathType pathLayout;
    frc::Trajectory trajectory;

    printf("Distance: %3f Angle: %3f\n", distance, angle);

    if (finished)
        return convertArrayToTrajectory(Finished, sizeof Finished / sizeof Finished);

    if (distance != -1.0)
    {
        if (angle < 0)
        {
            if (bluePath) 
                pathLayout = kBlueB;
            else
                pathLayout = kRedB;
        }
            
        else
        if (angle > 0)
        {
            if (bluePath)
                pathLayout = kBlueA;
            else
                pathLayout = kRedA;
        }
        finished = true;
    }
    else
    {
        bluePath = true;
        return convertArrayToTrajectory(Blue, sizeof Blue / sizeof Blue[0]);
    }
    
    switch (pathLayout)
    {
    case kRedA:
        trajectory = convertArrayToTrajectory(RedA, sizeof RedA / sizeof RedA[0]);
        break;
    case kBlueA:
        trajectory = convertArrayToTrajectory(BlueA, sizeof BlueA / sizeof BlueA[0]);
        break;
    case kRedB:
        trajectory = convertArrayToTrajectory(RedB, sizeof RedB / sizeof RedB[0]);
        break;
    case kBlueB:
        trajectory = convertArrayToTrajectory(BlueB, sizeof BlueB / sizeof BlueB[0]);
        break;
    }

    return trajectory;
}

frc2::Command *RobotContainer::GetDriveTestCommand(Direction direction)
{

    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drive.kDriveKinematics);
    frc::Trajectory exampleTrajectory;

    m_gyro.ZeroHeading();
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
