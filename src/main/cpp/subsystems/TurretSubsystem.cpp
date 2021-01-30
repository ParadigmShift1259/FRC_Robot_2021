
#include "subsystems/TurretSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace TurretConstants;

TurretSubsystem::TurretSubsystem() 
    : m_turretmotor(TurretConstants::kMotorPort)
{
    m_turretmotor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, TurretConstants::kTimeout);
    m_turretmotor.SetNeutralMode(NeutralMode::Brake);
    m_turretmotor.SetSensorPhase(TurretConstants::kSensorPhase);
    m_turretmotor.SetInverted(TurretConstants::kInverted);

    m_turretmotor.Config_kP(0, TurretConstants::kP, TurretConstants::kTimeout);
    m_turretmotor.Config_kI(0, TurretConstants::kI, TurretConstants::kTimeout);
    m_turretmotor.Config_kD(0, TurretConstants::kD, TurretConstants::kTimeout);

    m_turretmotor.ConfigNominalOutputForward(TurretConstants::kMinOut, TurretConstants::kTimeout);
    m_turretmotor.ConfigNominalOutputReverse(TurretConstants::kMinOut * -1.0, TurretConstants::kTimeout);
    m_turretmotor.ConfigPeakOutputForward(TurretConstants::kMaxOut, TurretConstants::kTimeout);
    m_turretmotor.ConfigPeakOutputReverse(TurretConstants::kMaxOut * -1.0, TurretConstants::kTimeout);
    //m_turretmotor.ConfigClosedloopRamp()
    m_turretmotor.ConfigAllowableClosedloopError(0, DegreesToTicks(TurretConstants::kDegreeStopRange), TurretConstants::kTimeout);

    m_turretmotor.SetSelectedSensorPosition(DegreesToTicks(TurretConstants::kStartingPositionDegrees), 0, TurretConstants::kTimeout);
}

void TurretSubsystem::Periodic()
{
    SmartDashboard::PutNumber("Turret Current Angle", TicksToDegrees(m_turretmotor.GetSelectedSensorPosition()));
    SmartDashboard::PutNumber("Turret Desired Angle", m_turretmotor.GetClosedLoopTarget());   
}

void TurretSubsystem::TurnTo(double angle)
{
    // safeguard
    angle = Util::ZeroTo360Degs(angle);
    // Turret is not set if desired angle is within the deadzone area
    if (angle >= TurretConstants::kMinAngle && angle <= TurretConstants::kMaxAngle)
    {
        m_turretmotor.Set(ControlMode::Position, DegreesToTicks(angle));
    }
}

void TurretSubsystem::TurnToRobot(double robotAngle)
{
    double angle = robotAngle - TurretConstants::kTurretToRobotAngleOffset;
    TurnTo(Util::ZeroTo360Degs(angle));
}

void TurretSubsystem::TurnToField(double desiredAngle, double gyroAngle)
{
    // safeguard
    desiredAngle = Util::ZeroTo360Degs(desiredAngle);
    gyroAngle = Util::ZeroTo360Degs(gyroAngle;
    // The difference between the field and robot is the desired angle to set relative to the robot
    double angle = gyroAngle - desiredAngle;
    TurnToRobot(Util::ZeroTo360Degs(angle));
}

void TurretSubsystem::TurnToRelative(double angle)
{
    // TicksToDegrees(m_turretmotor.GetSelectedSensorPosition()
}

bool TurretSubsystem::isAtSetpoint()
{
    return fabs(m_turretmotor.GetClosedLoopError()) <= DegreesToTicks(TurretConstants::kDegreeStopRange);
}

double TurretSubsystem::TicksToDegrees(double ticks)
{
    double rev = ticks / TurretConstants::kTicksPerRev;
    double turretrev = rev * TurretConstants::kMotorRevPerRev;
    return turretrev * TurretConstants::kDegreesPerRev;
}


double TurretSubsystem::DegreesToTicks(double degrees)
{
    double turretrev = degrees / TurretConstants::kDegreesPerRev;
    double rev = turretrev / TurretConstants::kMotorRevPerRev;
    return rev * TurretConstants::kTicksPerRev;
}