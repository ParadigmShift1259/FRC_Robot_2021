
#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace IntakeConstants;
using namespace std;
using namespace frc;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(IntakeConstants::kMotorPort)
{
    SmartDashboard::PutNumber("Intake Motor Speed", 0);
}

void IntakeSubsystem::Periodic()
{
    SmartDashboard::PutNumber("Intake Motor Speed", m_motor.GetMotorOutputPercent());
}

void IntakeSubsystem::Run(double speed) 
{
    m_motor.Set(ControlMode::PercentOutput, speed * IntakeConstants::kMotorReverseConstant);
}