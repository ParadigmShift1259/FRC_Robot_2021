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

#include <iostream>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"

class RobotContainer
{
public:
    RobotContainer();

    void Periodic();

    frc2::Command *GetAutonomousCommand();

private:
    double Deadzone(double inputValue, double deadzone)
    {
        // If the input is small return 0
        return abs(inputValue) <= deadzone ? 0 : inputValue;
    }    

    // The driver's controller
    frc::XboxController m_primaryController{OIConstants::kPrimaryControllerPort};

    DriveSubsystem m_drive;

    void SetDefaultCommands();
    void ConfigureButtonBindings();

    bool m_fieldRelative = false;
};
