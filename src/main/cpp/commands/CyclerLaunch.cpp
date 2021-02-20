#include "commands/CyclerLaunch.h"

using namespace CyclerConstants;

CyclerLaunch::CyclerLaunch(CyclerSubsystem* subsystem, bool* turretready, bool* cyclerready)
 : m_cycler(subsystem)
 , m_turretready(turretready)
 , m_cyclerready(cyclerready)
{
  AddRequirements({subsystem});
}

void CyclerLaunch::Execute() {
    if (*m_turretready && *m_cyclerready) {
        m_cycler->SetTurnTable(kTurnTableSpeed);
        m_cycler->SetFeeder(kFeederSpeed);
    }
}