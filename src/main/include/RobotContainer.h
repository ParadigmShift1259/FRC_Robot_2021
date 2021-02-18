/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "Logger.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/HoodSubsystem.h"

#include "commands/DriveForward.h"
#include "commands/FlywheelIdle.h"
#include "commands/FlywheelRamp.h"
#include "commands/HoodRaise.h"
#include "commands/IntakeIngest.h"
#include "commands/IntakeRelease.h"
#include "commands/TurretControl.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
    RobotContainer(Logger& log);

    frc2::Command *GetAutonomousCommand();

    void ResetLog() { m_drive.ResetLog(); }

    void Periodic();

private:
    double Deadzone(double inputValue, double deadzone)
    {
        if (fabs(inputValue) <= deadzone)
        {
            // If the input is small return 0
            return 0.0;
        }
        
        // Otherwise hand back the input value
        return inputValue;
    }    
    
    Logger& m_log;

    // The driver's controller
    frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

    // The robot's subsystems and commands are defined here...

    // The robot's subsystems
    DriveSubsystem m_drive;
    HoodSubsystem m_hood;
    FlywheelSubsystem m_flywheel;


    // m_units::meters_per_second_t m_xInput;      //!< Last x input value
    // units::meters_per_second_t m_yInput;        //!< Last y input value
    // units::radians_per_second_t m_rotInput;     //!< Last rotation input value

     // The chooser for the autonomous routin;
    frc::SendableChooser<frc2::Command *> m_chooser;

    nt::NetworkTableEntry m_inputXentry;
    nt::NetworkTableEntry m_inputYentry;
    nt::NetworkTableEntry m_inputRotentry;

    void ConfigureButtonBindings();

    bool m_fieldRelative = false;

    frc2::InstantCommand m_enableFieldRelative{[this] () { m_fieldRelative = true; }};
    frc2::InstantCommand m_disableFieldRelative{[this] () { m_fieldRelative = false; }};
};
