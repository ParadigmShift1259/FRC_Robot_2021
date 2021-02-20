
#include "subsystems/ClimberSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace ClimberConstants;
using namespace std;
using namespace frc;

ClimberSubsystem::ClimberSubsystem() 
    : m_motor(ClimberConstants::kMotorPort)
{
    SmartDashboard::PutNumber("Climber Motor Speed", 0);
}

void ClimberSubsystem::Periodic()
{
    SmartDashboard::PutNumber("Climber Motor Speed", m_motor.GetMotorOutputPercent());
}

void ClimberSubsystem::Run(double speed) 
{
    m_motor.Set(ControlMode::PercentOutput, speed * ClimberConstants::kMotorReverseConstant);
}