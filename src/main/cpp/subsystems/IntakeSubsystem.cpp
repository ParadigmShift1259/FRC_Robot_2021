
#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace IntakeConstants;
using namespace std;
using namespace frc;

IntakeSubsystem::IntakeSubsystem(const int& m_lowPrioritySkipCount) 
    : m_motor(kMotorPort)
    , m_lowPrioritySkipCount(m_lowPrioritySkipCount)
{

}

void IntakeSubsystem::Periodic()
{
    if (m_lowPrioritySkipCount % 10 == 0)   // 5 per second
    {
        SmartDashboard::PutNumber("D_I_Motor", m_motor.Get());
    }
}

void IntakeSubsystem::Set(double speed) 
{
    m_motor.Set(speed * kMotorReverseConstant);
}