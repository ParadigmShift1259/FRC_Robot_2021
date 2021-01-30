
#include "subsystems/FlywheelSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace FlywheelConstants;

FlywheelSubsystem::FlywheelSubsystem() 
    : m_flywheelmotor(FlywheelConstants::kMotorPort, CANSparkMax::MotorType::kBrushless)
    , m_flywheelPID(m_flywheelmotor)
    , m_flywheelencoder(m_flywheelmotor)
    , m_flywheelFF(
        FlywheelConstants::kS * 1_V, 
        FlywheelConstants::kV * 1_V * 1_s / 1_m, 
        FlywheelConstants::kA * 1_V * 1_s * 1_s / 1_m
    )
{
    m_flywheelmotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_flywheelmotor.SetClosedLoopRampRate(FlywheelConstants::kRampRate);

    m_flywheelPID.SetP(FlywheelConstants::kP, 0);
    m_flywheelPID.SetI(FlywheelConstants::kI, 0);
    m_flywheelPID.SetD(FlywheelConstants::kD, 0);
    m_flywheelPID.SetOutputRange(FlywheelConstants::kMinOut, FlywheelConstants::kMaxOut);
}

void FlywheelSubsystem::Periodic()
{
}

void FlywheelSubsystem::SetRPM(double rpm)
{
    // Ignore PIDF feedforward and substitute WPILib's SimpleMotorFeedforward class
    double FF = m_flywheelFF.Calculate(rpm * FlywheelConstants::kMPSPerRPM * 1_mps).to<double>();
    m_flywheelPID.SetFF(0);

    m_flywheelPID.SetReference(rpm, ControlType::kVelocity, 0, FF);
}