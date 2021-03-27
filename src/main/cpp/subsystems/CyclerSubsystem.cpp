#include "subsystems/CyclerSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CyclerConstants;

CyclerSubsystem::CyclerSubsystem()
    : m_feedermotor(kFeederPort, CANSparkMax::MotorType::kBrushless)
    , m_turntablemotor(kTurnTablePort)
    , m_sensor(0)
    , m_interruptsenabled(false)
{
    m_turntablemotor.SetNeutralMode(NeutralMode::Brake);
    m_turntablemotor.SetInverted(kTurnTableInverted);
    m_turntablemotor.ConfigOpenloopRamp(kTurnTableRampRate, kTimeout);
    m_feedermotor.SetInverted(kFeederInverted);

    m_triggeredsensor = false;

    m_sensor.RequestInterrupts(
        [this] (frc::InterruptableSensorBase::WaitResult result) {
            // printf("Interrupt\n");
            if (result == frc::InterruptableSensorBase::WaitResult::kFallingEdge) {
                m_triggeredsensor = true;
                EndDetection();
            }
        });
    m_sensor.SetUpSourceEdge(false, true);
}

void CyclerSubsystem::Periodic()
{
    SmartDashboard::PutBoolean("D_C_Sensor", kSensorInvert ? !m_sensor.Get() : m_sensor.Get());
    SmartDashboard::PutBoolean("D_C_SensorFlag", m_triggeredsensor);
    m_sensor.ClearError();
}

void CyclerSubsystem::SetFeeder(double speed)
{
    m_feedermotor.Set(speed);
}

void CyclerSubsystem::SetTurnTable(double speed)
{
    m_turntablemotor.Set(ControlMode::PercentOutput, speed);
}

void CyclerSubsystem::StartDetection() {
    // printf("START DETECTION\n");
    //m_triggeredsensor = false;
    m_sensor.EnableInterrupts();
}

void CyclerSubsystem::EndDetection() {
    //m_sensor.CancelInterrupts();
    m_sensor.DisableInterrupts();
}

bool CyclerSubsystem::AtPosition()
{
    return m_triggeredsensor;
}

void CyclerSubsystem::ResetSensor()
{
    //EndDetection();
    m_triggeredsensor = false;
}