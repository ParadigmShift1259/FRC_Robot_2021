#include "subsystems/CyclerSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CyclerConstants;

CyclerSubsystem::CyclerSubsystem()
    : m_feedermotor(kFeederPort, CANSparkMax::MotorType::kBrushless)
    , m_turntablemotor(kTurnTablePort)
    , m_sensor(0)
{
    m_turntablemotor.SetNeutralMode(NeutralMode::Brake);
    m_turntablemotor.SetInverted(kTurnTableInverted);
    m_turntablemotor.ConfigOpenloopRamp(kTurnTableRampRate, kTimeout);
    m_feedermotor.SetInverted(kFeederInverted);
}

void CyclerSubsystem::Periodic()
{
    SmartDashboard::PutBoolean("D_C_SensorV2", m_sensor.Get());
    SmartDashboard::PutNumber("D_C_Sensor", m_sensor.Get());
}

void CyclerSubsystem::SetFeeder(double speed)
{
    m_feedermotor.Set(speed);
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