#include "commands/CyclerAgitation.h"
#include "Constants.h"

/// From an old WPILib repo (not official)
// #include "timer/Timer.h"

using namespace CyclerConstants;

CyclerAgitation::CyclerAgitation(CyclerSubsystem* subsystem)
 : m_cycler(subsystem)
 , m_timer()
 {
  AddRequirements({subsystem});
}

void CyclerAgitation::Initialize(){
    m_timer.Start();
}

void CyclerAgitation::Execute() {
    if (m_timer.Get() <= kTimePassed * 1) {
        m_cycler->SetTurnTable(kTurnTableSpeed);
    }
    else if (m_timer.Get() <= kTimePassed * 2) {
        m_cycler->SetTurnTable(0);
    }
    else if (m_timer.Get() <= kTimePassed * 3) {
        m_cycler->SetTurnTable(kTurnTableSpeed * -1.0);
    }
    else if (m_timer.Get() <= kTimePassed * 4) {
        m_cycler->SetTurnTable(0);
    }
    else if (m_timer.Get() > kTimePassed * 4) {
        m_timer.Reset();
    }
}