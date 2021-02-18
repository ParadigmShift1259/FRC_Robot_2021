#include "subsystems/CyclerSubsystem.h"

using namespace CyclerConstants;

CyclerSubsystem::CyclerSubsystem()
    : m_feedermotor(CyclerConstants::kFeederPort), m_turntablemotor(CyclerConstants::kTurnTablePort)
{
    
    m_turntablemotor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, CyclerConstants::kTimeout);
    m_turntablemotor.SetNeutralMode(NeutralMode::Brake);
    m_turntablemotor.SetSensorPhase(CyclerConstants::kSensorPhase);
    m_turntablemotor.SetInverted(CyclerConstants::kInverted);

    m_turntablemotor.SetSelectedSensorPosition(DegreesToTicks(CyclerConstants::kStartingPositionDegrees), 0, CyclerConstants::kTimeout);
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
    return m_turntablemotor.SetSelectedSensorPosition(DegreesToTicks(CyclerConstants::kStartingPositionDegrees), 0, CyclerConstants::kTimeout);
}

double CyclerSubsystem::GetAngle()
{
    return TicksToDegrees(m_turntablemotor.SetSelectedSensorPosition(DegreesToTicks(CyclerConstants::kStartingPositionDegrees), 0, CyclerConstants::kTimeout));
}

double CyclerSubsystem::TicksToDegrees(double ticks)
{
    double rev = ticks / CyclerConstants::kTicksPerRev;
    double turntablerev = rev * CyclerConstants::kMotorRevPerRev;
    return turntablerev * CyclerConstants::kDegreesPerRev;
}


double CyclerSubsystem::DegreesToTicks(double degrees)
{
    double turntablerev = degrees / CyclerConstants::kDegreesPerRev;
    double rev = turntablerev / CyclerConstants::kMotorRevPerRev;
    return rev * CyclerConstants::kTicksPerRev;
}