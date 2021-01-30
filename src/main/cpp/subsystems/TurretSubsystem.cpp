
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
     m_turretmotor.ConfigAllowableClosedloopError(0, DegreesToTicks(TurretConstants::kDegreeStopRange), TurretConstants::kTimeout);
}

void TurretSubsystem::Periodic()
{
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