/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot()
    : m_log(false)
    , m_container(m_log)
{
}

void Robot::RobotInit()
{
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit()
{
    m_log.logMsg(eInfo, __func__, "Disabling");
    m_log.closeLog();
}

void Robot::DisabledPeriodic()
{
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
    m_container.ResetLog();
    m_log.openLog();
    m_log.logMsg(eInfo, __func__, "Starting Autonomous");

    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
    m_container.ResetLog();
    m_log.openLog();
    m_log.logMsg(eInfo, __func__, "Starting Teleop");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }

    /* Used for Debugging RotationDrive PID and SwerveModule Turn PID

    SmartDashboard::PutNumber("kRotationP", DriveConstants::kRotationP);
    SmartDashboard::PutNumber("kRotationI", DriveConstants::kRotationI);
    SmartDashboard::PutNumber("kRotationD", DriveConstants::kRotationD);
    SmartDashboard::PutNumber("kMaxRotation", DriveConstants::kMaxAbsoluteRotationSpeed);
    SmartDashboard::PutNumber("kMaxRotationSpeed", DriveConstants::kMaxAbsoluteTurnableSpeed);
    
    SmartDashboard::PutNumber("TEST_TurnP", DriveConstants::kTurnP);
    SmartDashboard::PutNumber("TEST_TurnI", DriveConstants::kTurnI);
    SmartDashboard::PutNumber("TEST_TurnD", DriveConstants::kTurnD);

    */
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic()
{
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
