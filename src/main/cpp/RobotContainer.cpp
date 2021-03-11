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
#define SUBSYSTEMS

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

RobotContainer::RobotContainer(Logger& log)
    : m_log(log)
    , m_drive(log)
#ifdef SUBSYSTEMS
    , m_flywheel()
    , m_turret()
    // , m_hood()
    , m_intake()
    , m_cycler()
    // , m_vision()
    // , m_climber()
#endif
{
    // Initialize all of your commands and subsystems here
    m_fieldRelative = false;

    // Configure the button bindings
    ConfigureButtonBindings();
    SetDefaultCommands();

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

void RobotContainer::SetDefaultCommands()
{
    // m_drive.SetDefaultCommand(frc2::RunCommand(
    //     [this] {
    //         // up is xbox joystick y pos
    //         // left is xbox joystick x pos
    //         auto xInput = Deadzone(m_driverController.GetY(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneX);
    //         auto yInput = Deadzone(m_driverController.GetX(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneY);
    //         auto rotInput = Deadzone(m_driverController.GetX(frc::GenericHID::kRightHand) * -1.0, OIConstants::kDeadzoneRot);
    //         auto xRot = m_driverController.GetY(frc::GenericHID::kRightHand) * -1.0;
    //         auto yRot = m_driverController.GetX(frc::GenericHID::kRightHand) * -1.0;
    //         if (Deadzone(sqrt(pow(xRot, 2) + pow(yRot, 2)), OIConstants::kDeadzoneAbsRot) == 0) {
    //             xRot = 0;
    //             yRot = 0;
    //         }

    //         m_inputXentry.SetDouble(xInput);
    //         m_inputYentry.SetDouble(yInput);
    //         m_inputRotentry.SetDouble(rotInput);

    //         if (m_fieldRelative)
    //         {
    //             m_drive.RotationDrive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
    //                         units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
    //                         xRot,
    //                         yRot,
    //                         m_fieldRelative);
    //         }
    //         else 
    //         {
    //             m_drive.Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
    //                         units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
    //                         units::radians_per_second_t(rotInput),
    //                         m_fieldRelative);
    //         }

    //     },
    //     {&m_drive}
    // ));

    // m_drive.SetDefaultCommand(
    //     DriveDefault(&m_drive, 
    //         [this] {
    //             double x = Deadzone(m_driverController.GetY(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneX);
    //             m_inputXentry.SetDouble(x);
    //             return x;
    //         },
    //         [this] {
    //             double y = Deadzone(m_driverController.GetX(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneY);
    //             m_inputYentry.SetDouble(y);
    //             return y;
    //         },
    //         [this] {
    //             double rot = Deadzone(m_driverController.GetX(frc::GenericHID::kRightHand) * -1.0, OIConstants::kDeadzoneRot);
    //             m_inputRotentry.SetDouble(rot);
    //             return rot;
    //         },
    //         [this] {
    //             return m_driverController.GetY(frc::GenericHID::kRightHand) * -1.0;
    //         }, 
    //         [this] {
    //             return m_driverController.GetX(frc::GenericHID::kRightHand) * -1.0;
    //         },
    //         [this] {
    //             return m_fieldRelative;
    //         }
    //     )
    // );

    #ifdef SUBSYSTEMS

    // m_turret.SetDefaultCommand(
    //     frc2::RunCommand(
    //         [this] {
    //             m_turret.TurnTo(45);
    //         }, {&m_turret}
    //     )
    // );

    // m_flywheel.SetDefaultCommand(
    //     frc2::RunCommand(
    //         [this] {
    //             m_flywheel.SetRPM(FlywheelConstants::kIdleRPM);
    //         }, {&m_flywheel}
    //     )
    // );

    // m_hood.SetDefaultCommand(
    //     frc2::RunCommand(
    //         [this] {
    //             m_hood.Set(0);
    //         }, {&m_hood}
    //     )
    // );
    
    // m_climber.SetDefaultCommand(
    //     frc2::RunCommand(
    //         [this] {
    //             m_climber.Run(0);
    //         }, {&m_climber}
    //     )
    // );

    m_cycler.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                m_cycler.SetFeeder(0);
                m_cycler.SetTurnTable(0);
            }, {&m_cycler}
        )
    );

    #endif

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

    // Triggers Fire sequence
    // frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY).WhenHeld(
    //     Fire(&m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision)
    // );

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

    #ifdef SUBSYSTEMS

    // Increments / Decrements a test power value for TestCommands()
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY).WhenPressed(
        frc2::InstantCommand(    
        [this] {
            m_testPower += 0.05;
            SmartDashboard::PutNumber("TEST_testPower", m_testPower);
        },
        {}
        )
    );

    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kX).WhenPressed(
        frc2::InstantCommand(    
        [this] {
            m_testPower -= 0.05;
            SmartDashboard::PutNumber("TEST_testPower", m_testPower);
        },
        {}
        )
    );

    bool temporaryBoolean = false;

    // Runs sequence of tests for motors based on iterator and a power
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA).WhenPressed(
        /*
        frc2::InstantCommand(    
            [this] {
                printf("scheduling flywheel RPM ONE TIME\n");
                m_flywheel.SetRPM(m_testPower * 1000.0);
            },
            {&m_flywheel}
        )*/
        CyclerPrepare(&m_cycler, &temporaryBoolean)
    );

    #endif 
}

frc2::InstantCommand RobotContainer::TestCommands()
{
    printf("%d", (int) SmartDashboard::GetNumber("TEST_testNumber", 0));
    m_testNumber = (int) SmartDashboard::GetNumber("TEST_testNumber", 0);

    switch(m_testNumber) {
    case 0:
        printf("case 0");
        return 
        frc2::InstantCommand(    
            [this] {
                m_intake.Set(m_testPower);
            },
            {&m_intake}
        );
    case 1:
        printf("case 1");
        return
        frc2::InstantCommand(    
            [this] {
                m_cycler.SetFeeder(m_testPower);
            },
            {&m_cycler}
        );
    case 2:
        printf("case 2");
        return 
        frc2::InstantCommand(    
            [this] {
                m_cycler.SetTurnTable(m_testPower);
            },
            {&m_cycler}
        );
    case 3:
        printf("case 3");
        return
        frc2::InstantCommand(    
            [this] {
                m_flywheel.SetRPM(m_testPower * 1000.0);
            },
            {&m_flywheel}
        );
        break;
    case 4:
        printf("case 4");
        return 
        frc2::InstantCommand(    
            [this] {
                m_hood.Set(m_testPower);
            },
            {&m_hood}
        );
        break;
    case 5:
        printf("case 5");
        return
        frc2::InstantCommand(    
            [this] {
                m_climber.Run(m_testPower);
            },
            {&m_climber}
        );
    }
    printf("Default");
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
    //m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

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

// /*
    wpi::SmallString<64> deployDirectory;
    frc::filesystem::GetDeployDirectory(deployDirectory);
    wpi::sys::path::append(deployDirectory, "paths/output"); //Has the projects that are created in meters
    wpi::sys::path::append(deployDirectory, "AutoNavBarrel.wpilib.json");

    frc::Trajectory exampleTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    /*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        TestTrajLine,
        config
    );
    */
    // std::cout << "Number of Trajectory States: \n" << exampleTrajectory.States().size();
    
    // for (int i = 0; i < exampleTrajectory.States().size(); i++)
    // {
    //     std::cout << "i " << i;
    //     std::cout << " x = " << exampleTrajectory.States()[i].pose.X();
    //     std::cout << " y = " << exampleTrajectory.States()[i].pose.Y();
    //     std::cout << " velocity = " << exampleTrajectory.States()[i].velocity;
    //     std::cout << " acceleration = " << exampleTrajectory.States()[i].acceleration;
    //     std::cout << " theta = " << exampleTrajectory.States()[i].pose.Rotation().Degrees() << std::endl;
    // }
// */

 /*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        TestTrajCircle2,
        config
    );
//    std::cout << "Number of Trajectory States: \n" << exampleTrajectory.States().size();
    
    // Transform2d rot90 = Transform2d(Pose2d(0_m,0_m,0_deg), Pose2d(0_m,0_m,90_deg));
    // Transform2d rot90 = Transform2d(Translation2d(), 90_deg);
    // for (int i = 0; i < exampleTrajectory.States().size(); i++)
    //     {
    //     std::cout << "pose " << i << " theta = " << exampleTrajectory.States()[i].pose.Rotation().Degrees();
    //     exampleTrajectory.States()[i].pose.TransformBy(rot90); 
    //     std::cout << " modified theta = " << exampleTrajectory.States()[i].pose.Rotation().Degrees() <<  "\n";        
    //     }
        
// */

/*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        AutoNavBarrel,
        config
    );
    // std::cout << "Number of Trajectory States: \n" << exampleTrajectory.States().size();
    
    // for (int i = 0; i < exampleTrajectory.States().size(); i++)
    // {
    //     std::cout << "i " << i;
    //     std::cout << " x = " << exampleTrajectory.States()[i].pose.X();
    //     std::cout << " y = " << exampleTrajectory.States()[i].pose.Y();
    //     std::cout << " velocity = " << exampleTrajectory.States()[i].velocity;
    //     std::cout << " acceleration = " << exampleTrajectory.States()[i].acceleration;
    //     std::cout << " theta = " << exampleTrajectory.States()[i].pose.Rotation().Degrees() << std::endl;
    // }
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
