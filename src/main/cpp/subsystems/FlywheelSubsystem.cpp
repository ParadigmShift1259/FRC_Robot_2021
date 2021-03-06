
#include "subsystems/FlywheelSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace FlywheelConstants;

// Enable to tune the flywheel constants
#define TUNE_FLYHEEL

// Removes deprecated warning for CANEncoder and CANPIDController
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

FlywheelSubsystem::FlywheelSubsystem() 
    : m_flywheelmotor(kMotorPort, CANSparkMax::MotorType::kBrushless)
    , m_flywheelPID(m_flywheelmotor)
    , m_flywheelencoder(m_flywheelmotor)
    , m_flywheelFF(
        kS * 1_V, 
        kV * 1_V * 1_s / 1_m, 
        kA * 1_V * 1_s * 1_s / 1_m
    )
{
    m_flywheelmotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_flywheelmotor.SetClosedLoopRampRate(kRampRate);

    m_flywheelPID.SetP(kP, 0);
    m_flywheelPID.SetI(kI, 0);
    m_flywheelPID.SetD(kD, 0);
    m_flywheelPID.SetOutputRange(kMinOut, kMaxOut);

    m_setpoint = 0;

    #ifdef TUNE_FLYWHEEL
    SmartDashboard::PutNumber("T_F_S", 0);
    SmartDashboard::PutNumber("T_F_V", 0);
    SmartDashboard::PutNumber("T_F_A", 0);
    SmartDashboard::PutNumber("T_F_P", 0);
    SmartDashboard::PutNumber("T_F_I", 0);
    SmartDashboard::PutNumber("T_F_D", 0);
    #endif
}

#pragma GCC diagnostic pop

void FlywheelSubsystem::Periodic()
{
    #ifdef TUNE_FLYWHEEL
    double s = SmartDashboard::GetNumber("T_F_S", 0);
    double v = SmartDashboard::GetNumber("T_F_V", 0);
    double a = SmartDashboard::GetNumber("T_F_A", 0);
    double p = SmartDashboard::GetNumber("T_F_P", 0);
    double i = SmartDashboard::GetNumber("T_F_I", 0);
    double d = SmartDashboard::GetNumber("T_F_D", 0);
    m_flywheelFF.kS = s * 1_V;
    m_flywheelFF.kV = v * 1_V * 1_s / 1_m;
    m_flywheelFF.kA = a * 1_V * 1_s * 1_s / 1_m;
    m_flywheelPID.SetP(p, 0);
    m_flywheelPID.SetI(i, 0);
    m_flywheelPID.SetD(d, 0);
    #endif
}

void FlywheelSubsystem::SetRPM(double rpm)
{
    // Ignore PIDF feedforward and substitute WPILib's SimpleMotorFeedforward class
    double FF = m_flywheelFF.Calculate(rpm * kMPSPerRPM * 1_mps).to<double>();
    m_flywheelPID.SetFF(0);

    m_setpoint = rpm;
    m_flywheelPID.SetReference(m_setpoint, ControlType::kVelocity, 0, FF);
}

bool FlywheelSubsystem::isAtRPM() {
    return fabs(m_flywheelencoder.GetVelocity() - m_setpoint) >= kAllowedError;
}