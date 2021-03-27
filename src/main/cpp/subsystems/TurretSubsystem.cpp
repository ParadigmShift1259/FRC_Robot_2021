
#include "subsystems/TurretSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace TurretConstants;

TurretSubsystem::TurretSubsystem() 
    : m_turretmotor(kMotorPort)
{
    m_turretmotor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeout);
    m_turretmotor.SetNeutralMode(NeutralMode::Brake);
    m_turretmotor.SetSensorPhase(kSensorPhase);
    m_turretmotor.SetInverted(kInverted);

    m_turretmotor.Config_kP(0, kP, kTimeout);
    m_turretmotor.Config_kI(0, kI, kTimeout);
    m_turretmotor.Config_kD(0, kD, kTimeout);

    m_turretmotor.ConfigNominalOutputForward(kMinOut, kTimeout);
    m_turretmotor.ConfigNominalOutputReverse(kMinOut * -1.0, kTimeout);
    m_turretmotor.ConfigPeakOutputForward(kMaxOut, kTimeout);
    m_turretmotor.ConfigPeakOutputReverse(kMaxOut * -1.0, kTimeout);
    //m_turretmotor.ConfigClosedloopRamp()
    m_turretmotor.ConfigAllowableClosedloopError(0, DegreesToTicks(kDegreePIDStopRange), kTimeout);

    m_turretmotor.SetSelectedSensorPosition(DegreesToTicks(kStartingPositionDegrees), 0, kTimeout);
    m_turretmotor.Set(ControlMode::Position, DegreesToTicks(kStartingPositionDegrees));
    m_currentAngle = kStartingPositionDegrees;

    frc::SmartDashboard::PutNumber("T_T_P", kP);
    frc::SmartDashboard::PutNumber("T_T_I", kI);
    frc::SmartDashboard::PutNumber("T_T_D", kD);
    frc::SmartDashboard::PutNumber("T_T_Max", kMaxOut);
}

void TurretSubsystem::Periodic()
{
    SmartDashboard::PutNumber("D_T_CAngle", TicksToDegrees(m_turretmotor.GetSelectedSensorPosition()));
    SmartDashboard::PutNumber("D_T_DAngle", TicksToDegrees(m_turretmotor.GetClosedLoopTarget()));
    SmartDashboard::PutNumber("D_T_Error", TicksToDegrees(m_turretmotor.GetClosedLoopError(0)));
    SmartDashboard::PutNumber("D_T_Output", m_turretmotor.GetMotorOutputPercent());
    m_turretmotor.Set(ControlMode::Position, DegreesToTicks(m_currentAngle));
}

void TurretSubsystem::TurnTo(double angle)
{
    // safeguard
    angle = Util::ZeroTo360Degs(angle);
    // Turret is not set if desired angle is within the deadzone area
    if (angle >= kMinAngle && angle <= kMaxAngle)
        m_currentAngle = angle;
}

void TurretSubsystem::TurnToRobot(double robotAngle)
{
    double angle = robotAngle - kTurretToRobotAngleOffset;
    TurnTo(Util::ZeroTo360Degs(angle));
}

void TurretSubsystem::TurnToField(double desiredAngle, double gyroAngle)
{
    // safeguard
    desiredAngle = Util::ZeroTo360Degs(desiredAngle);
    gyroAngle = Util::ZeroTo360Degs(gyroAngle);
    // The difference between the field and robot is the desired angle to set relative to the robot
    double angle = desiredAngle - gyroAngle;
    TurnToRobot(Util::ZeroTo360Degs(angle));
}

void TurretSubsystem::TurnToRelative(double angle)
{   
    double desiredAngle = TicksToDegrees(m_turretmotor.GetSelectedSensorPosition());
    angle = Util::ZeroTo360Degs(angle);
    desiredAngle += angle;
    desiredAngle = Util::ZeroTo360Degs(desiredAngle);
    TurnTo(desiredAngle);
}

bool TurretSubsystem::isAtSetpoint()
{
    return fabs(m_turretmotor.GetClosedLoopError()) <= DegreesToTicks(kDegreeStopRange);
}

void TurretSubsystem::SetNewPIDValues()
{
    /// CONFIRMED: OVERUNNING THE ENTIRE LOOP
    /// Bug
    double p = frc::SmartDashboard::GetNumber("T_T_P", kP);
    double i = frc::SmartDashboard::GetNumber("T_T_I", kI);
    double d = frc::SmartDashboard::GetNumber("T_T_D", kD);
    double max = frc::SmartDashboard::GetNumber("T_T_Max", kMaxOut);
    m_turretmotor.Config_kP(0, p, kTimeout);
    m_turretmotor.Config_kI(0, i, kTimeout);
    m_turretmotor.Config_kD(0, d, kTimeout);
    m_turretmotor.ConfigPeakOutputForward(max, kTimeout);
    m_turretmotor.ConfigPeakOutputReverse(max * -1.0, kTimeout);
}

double TurretSubsystem::TicksToDegrees(double ticks)
{
    double rev = ticks / kTicksPerRev;
    double turretrev = rev * kMotorRevPerRev;
    return turretrev * kDegreesPerRev;
}


double TurretSubsystem::DegreesToTicks(double degrees)
{
    double turretrev = degrees / kDegreesPerRev;
    double rev = turretrev / kMotorRevPerRev;
    return rev * kTicksPerRev;
}