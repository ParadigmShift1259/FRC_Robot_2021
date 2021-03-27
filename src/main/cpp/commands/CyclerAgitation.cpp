#include "commands/CyclerAgitation.h"
#include "Constants.h"

/// From an old WPILib repo (not official)
// #include "timer/Timer.h"

using namespace CyclerConstants;

CyclerAgitation::CyclerAgitation(CyclerSubsystem* subsystem, double speed)
 : m_cycler(subsystem)
 , m_timer()
 , m_speed(speed)
 {
  AddRequirements({subsystem});
}

void CyclerAgitation::Initialize()
{
    // Ensures that cycler position is not valid
    m_cycler->ResetSensor();
    m_timer.Start();
}

void CyclerAgitation::Execute()
{
    if (m_timer.Get() <= kTimePassed * 1) {
        m_cycler->SetTurnTable(m_speed);
    }
    else if (m_timer.Get() <= kTimePassed * 2) {
        m_cycler->SetTurnTable(0);
    }
    else if (m_timer.Get() <= kTimePassed * 3) {
        m_cycler->SetTurnTable(m_speed * -1.0);
    }
    else if (m_timer.Get() <= kTimePassed * 4) {
        m_cycler->SetTurnTable(0);
    }
    else if (m_timer.Get() > kTimePassed * 4) {
        m_timer.Reset();
    }
}