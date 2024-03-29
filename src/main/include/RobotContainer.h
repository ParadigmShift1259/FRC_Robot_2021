/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Filesystem.h>
#include <frc/XboxController.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc/geometry/Translation2d.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include "common/Util.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/CyclerSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

#include "commands/CyclerAgitation.h"
#include "commands/Fire.h"
#include "commands/CyclerFire.h"
#include "commands/CyclerIntakeAgitation.h"
#include "commands/CyclerPrepare.h"
#include "commands/Unjam.h"
#include "commands/IntakeRelease.h"

#include "Constants.h"
#include "Gyro.h"

#include <iostream>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include "SwerveControllerCommand2.h"

class RobotContainer
{
public:
    RobotContainer();

    void Periodic();

    enum AutoPath{kLeft3, kLeft8, kMid0, kMid5, kRight2, kTest};
    frc2::Command *GetAutonomousCommand(AutoPath path);
    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> GetSwerveCommand(double path[][6], int length, bool primaryPath);

    frc::SendableChooser<AutoPath> m_chooser;

    void ZeroDrive();
    
private:


    frc::XboxController m_primaryController{OIConstants::kPrimaryControllerPort};
    frc::XboxController m_secondaryController{OIConstants::kSecondaryControllerPort};

    // The robot's subsystems
    Gyro m_gyro;
    DriveSubsystem m_drive;
    FlywheelSubsystem m_flywheel;
    TurretSubsystem m_turret;
    HoodSubsystem m_hood;
    IntakeSubsystem m_intake;
    CyclerSubsystem m_cycler;
    VisionSubsystem m_vision;
    ClimberSubsystem m_climber;

    void SetDefaultCommands();
    void ConfigureButtonBindings();
    void PrintTrajectory(frc::Trajectory& trajectory);

    bool m_fieldRelative = true;
    bool m_turretready = false;
    bool m_firing = false;
    bool m_finished = false;
 
    bool m_isRedPath = false;
};
