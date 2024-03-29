/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include <frc2/command/button/NetworkButton.h>

#include "AutoPaths.h"

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
    , m_climber()
{
    m_fieldRelative = false;

    m_turretready = false;
    m_firing = false;
    m_finished = false;

    ConfigureButtonBindings();
    SetDefaultCommands();
    SmartDashboard::PutBoolean("WheelsForward", false);

    m_chooser.SetDefaultOption("Left 3", AutoPath::kLeft3);
    m_chooser.AddOption("Left 8 (Experimental)", AutoPath::kLeft8);
    m_chooser.AddOption("Middle 0", AutoPath::kMid0);
    m_chooser.AddOption("Middle 5", AutoPath::kMid5);
    m_chooser.AddOption("Right 2", AutoPath::kRight2);
    m_chooser.AddOption("Test", AutoPath::kTest);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void RobotContainer::Periodic()
{
    SmartDashboard::PutNumber("Gyro", m_gyro.GetHeading());
}

void RobotContainer::SetDefaultCommands()
{
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            // up is xbox joystick y pos
            // left is xbox joystick x pos
            auto xInput = Util::Deadzone(m_primaryController.GetY(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneX);
            auto yInput = Util::Deadzone(m_primaryController.GetX(frc::GenericHID::kLeftHand) * -1.0, OIConstants::kDeadzoneY);
            if (Util::Deadzone(sqrt(pow(xInput, 2) + pow(yInput, 2)), OIConstants::kDeadzoneXY) == 0) {
                xInput = 0;
                yInput = 0;
            }

            auto rotInput = Util::Deadzone(m_primaryController.GetX(frc::GenericHID::kRightHand) * -1.0, OIConstants::kDeadzoneRot);
            auto xRot = m_primaryController.GetY(frc::GenericHID::kRightHand) * -1.0;
            auto yRot = m_primaryController.GetX(frc::GenericHID::kRightHand) * -1.0;
            if (Util::Deadzone(sqrt(pow(xRot, 2) + pow(yRot, 2)), OIConstants::kDeadzoneAbsRot) == 0) {
                xRot = 0;
                yRot = 0;
            }

            // m_drive.Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
            //                 units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
            //                 units::angular_velocity::radians_per_second_t(rotInput),
            //                 m_fieldRelative);


            if (!m_fieldRelative)
            {
            m_drive.Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                            units::angular_velocity::radians_per_second_t(rotInput),
                            false);
            }
            else 
            {
            m_drive.Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                            units::angular_velocity::radians_per_second_t(rotInput),
                            true);
            }
        },
        {&m_drive}
    ));

    m_turret.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                auto turretXRot = m_secondaryController.GetY(frc::GenericHID::kRightHand) * -1.0;
                auto turretYRot = m_secondaryController.GetX(frc::GenericHID::kRightHand);
                if (Util::Deadzone(sqrt(pow(turretXRot, 2) + pow(turretYRot, 2)), OIConstants::kDeadzoneAbsRot) == 0) {
                    turretXRot = 0;
                    turretYRot = 0;
                }

                if (turretXRot == 0 && turretYRot == 0)
                {
                    m_turret.TurnToField(0);
                    // If vision is not active or vision is active but outside of auto targeting range
                    // if (!m_vision.GetActive() || fabs(m_vision.GetAngle()) > TurretConstants::kMaxAutoRelAngle)
                    // {
                    //     m_turret.TurnToField(0);
                    // }
                    // // Auto target with smaller auto targeting range
                    // else
                    // {
                    //     m_turret.TurnToRelative(m_vision.GetAngle(), TurretConstants::kMinAutoAngle, TurretConstants::kMaxAutoAngle);
                    // }
                }
                else 
                {
                    double rotPosition = atan2f(turretYRot, turretXRot);
                    rotPosition *= 360.0/Math::kTau; 
                    m_turret.TurnToRobot(rotPosition);
                } 
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
    
    m_climber.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                m_climber.Run(0);
            }, {&m_climber}
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

    m_intake.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                m_intake.Set(0);
            }, {&m_intake}
        )
    );

    m_flywheel.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                //y=1687.747+16.6111x-0.0649x^{2}+0.000091892342x^{3}
                double distance = m_vision.GetDistance();
                if (distance > VisionConstants::kMinHoneDistance && distance < VisionConstants::kMaxHoneDistance) {
                    double flywheelspeed = 1687.747 + 16.6111 * distance - 0.0649 * pow(distance, 2) + 0.000091892342 * pow(distance, 3);
                    flywheelspeed *= FlywheelConstants::kIdleHomingRPMMultiplier;
                    m_flywheel.SetRPM(flywheelspeed);
                }
                else {
                    m_flywheel.SetRPM(FlywheelConstants::kIdleRPM);
                }
                // m_flywheel.SetRPM(0);
            }, {&m_flywheel, &m_vision}
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

    /*
    * Joystick Controls (found in SetDefaultCommands)
    * Primary:
    * Left X, Y -> Strafe
    * Right X   -> Rotate
    * 
    * Secondary
    * Right X, Y, -45 to 45 -> Force move turret
    */

    /*
    * Button Controls
    * Primary:
    * Left Bumper (Hold)    -> Field Relative 
    * Right Bumper (Press)  -> Reset Gyro
    * Start (Hold)          -> Climbs
    * Back  (Hold)          -> Reverses Climb (WILL NOT BE IN PRODUCTION)
    * 
    * Secondary:
    * Y (Press)             -> Shoot
    * A (Hold)              -> Ingest and Agitate
    * Left Bumper (Hold)    -> Agitate harder
    * B (Hold)              -> Release
    * Back (Hold)           -> Emergency Unjam
    * 
    */

    // Primary
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

    // frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kBack).WhileHeld(
    //     frc2::InstantCommand(    
    //     [this] {
    //         m_climber.Run(ClimberConstants::kMotorSpeed);
    //     },
    //     {&m_climber}
    //     )
    // );

    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kStart).WhileHeld(
        frc2::InstantCommand(    
        [this] {
            m_climber.Run(ClimberConstants::kMotorSpeed * -1.0);
        },
        {&m_climber}
        )
    );

    // Secondary
    // Triggers Fire sequence
    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kY).WhenPressed(
        Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision,
             &m_turretready, &m_firing, &m_finished)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kA).WhenHeld(
        CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed)   
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kBumperLeft).WhenHeld(
        CyclerAgitation(&m_cycler, CyclerConstants::kTurnTableSpeedHigher)   
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kBumperRight).WhenPressed(
        frc2::InstantCommand([this] { m_turret.ResetPosition(); }, { &m_turret} )
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kA).WhenReleased(
        CyclerPrepare(&m_cycler, true).WithTimeout(CyclerConstants::kMaxCyclerTime)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kB).WhenHeld(
        IntakeRelease(&m_intake)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kBack).WhenHeld(
        Unjam(&m_cycler, &m_intake)
    );

    frc2::NetworkButton("SmartDashboard", "WheelsForward").WhenPressed(
        frc2::InstantCommand([this] { m_drive.WheelsForward(); }, { &m_drive} )        
    );
}

frc::Rotation2d GetDesiredRotation() { return frc::Rotation2d(0_deg); }

void RobotContainer::ZeroDrive()
{
    m_drive.Drive(units::meters_per_second_t(0.0),
                units::meters_per_second_t(0.0),
                units::radians_per_second_t(0.0), false);
}

frc2::Command *RobotContainer::GetAutonomousCommand(AutoPath path)
{
    switch(path)
    {
        case kLeft3:
            return new frc2::SequentialCommandGroup(
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 2.0),
                frc2::ParallelRaceGroup(
                    CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed),
                    std::move(GetSwerveCommand(left3, sizeof(left3) / sizeof(left3[0]), true))
                ),
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    },
                    {}
                ),
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        case kLeft8:
            return new frc2::SequentialCommandGroup(
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 2.0),
                frc2::ParallelRaceGroup(
                    CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed),
                    std::move(GetSwerveCommand(left8p1, sizeof(left8p1) / sizeof(left8p1[0]), true))
                ),
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 3),
                frc2::ParallelRaceGroup(
                    CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed),
                    std::move(GetSwerveCommand(left8p2, sizeof(left8p2) / sizeof(left8p2[0]), false))
                ),
                 frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    },
                    {}
                ),
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        case kMid0:
            return new frc2::SequentialCommandGroup(
                std::move(GetSwerveCommand(mid0, sizeof(mid0) / sizeof(mid0[0]), true)),
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    },
                    {}
                ),
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        case kMid5:
            return new frc2::SequentialCommandGroup(
                // frc2::ParallelCommandGroup(
                //     frc2::SequentialCommandGroup(
                //         std::move(GetSwerveCommand(mid5p0, sizeof(mid5p0) / sizeof(mid5p0[0]), true)),
                //         frc2::InstantCommand(
                //             [this]() {
                //                 m_drive.Drive(units::meters_per_second_t(0.0),
                //                             units::meters_per_second_t(0.0),
                //                             units::radians_per_second_t(0.0), false);
                //             },
                //             {}
                //         )
                //     ),
                //     Fire(&m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished).WithTimeout(6.0_s)
                // ),
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 1.8),
                frc2::ParallelRaceGroup(
                    CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed),
                    std::move(GetSwerveCommand(mid5, sizeof(mid5) / sizeof(mid5[0]), true))
                ),
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    },
                    {}
                ),
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        case kRight2:
            return new frc2::SequentialCommandGroup(
                frc2::ParallelRaceGroup(
                    CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed),
                    std::move(GetSwerveCommand(right2, sizeof(right2) / sizeof(right2[0]), true))
                ),
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    },
                    {}
                ),
                Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        default:
             return new frc2::SequentialCommandGroup(
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    },
                    {}
                )
            );
    }
}

frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> RobotContainer::GetSwerveCommand(double path[][6],  int length, bool primaryPath)
{
    frc::Trajectory exampleTrajectory = convertArrayToTrajectory(path, length);

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
    if(primaryPath)
        m_drive.ResetOdometry(exampleTrajectory.InitialPose());

    return swerveControllerCommand;
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
