#include "subsystems/CyclerSubsystem.h"

using namespace CyclerConstants;

CyclerSubsystem::CyclerSubsystem()
    : m_feedermotor(kFeederPort), m_turntablemotor(kTurnTablePort)
{
    
    m_turntablemotor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeout);
    m_turntablemotor.SetNeutralMode(NeutralMode::Brake);
    m_turntablemotor.SetSensorPhase(kSensorPhase);
    m_turntablemotor.SetInverted(kInverted);
    m_turntablemotor.ConfigOpenloopRamp(kTurnTableRampRate, kTimeout);

    m_turntablemotor.SetSelectedSensorPosition(DegreesToTicks(kStartingPositionDegrees), 0, kTimeout);
}

void CyclerSubsystem::SetFeeder(double speed)
{
    m_feedermotor.Set(ControlMode::PercentOutput, speed);
}

void CyclerSubsystem::SetTurnTable(double speed)
{
    m_turntablemotor.Set(ControlMode::PercentOutput, speed);
}

double CyclerSubsystem::GetPosition()
{
    return m_turntablemotor.SetSelectedSensorPosition(DegreesToTicks(kStartingPositionDegrees), 0, kTimeout);
}

double CyclerSubsystem::GetAngle()
{
    return TicksToDegrees(m_turntablemotor.SetSelectedSensorPosition(DegreesToTicks(kStartingPositionDegrees), 0, kTimeout));
}

double CyclerSubsystem::TicksToDegrees(double ticks)
{
    double rev = ticks / kTicksPerRev;
    double turntablerev = rev * kMotorRevPerRev;
    return turntablerev * kDegreesPerRev;
}


double CyclerSubsystem::DegreesToTicks(double degrees)
{
    double turntablerev = degrees / kDegreesPerRev;
    double rev = turntablerev / kMotorRevPerRev;
    return rev * kTicksPerRev;
}