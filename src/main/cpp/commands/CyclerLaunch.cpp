#include "commands/CyclerLaunch.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CyclerConstants;

CyclerLaunch::CyclerLaunch(CyclerSubsystem* subsystem, bool* turretready, bool* cyclerready, bool* finished)
 : m_cycler(subsystem)
 , m_turretready(turretready)
 , m_cyclerready(cyclerready)
 , m_finished(finished)
 , m_firing(false)
{
  AddRequirements({subsystem});
  *m_finished = false;
}

void CyclerLaunch::Initialize()
{
    m_timer.Reset();
    *m_finished = false;
}

void CyclerLaunch::Execute()
{
    if (*m_turretready && *m_cyclerready)
    {
        m_firing = true;
        m_timer.Start();
    }

    if (m_firing)
    {
        m_cycler->SetTurnTable(kTurnTableSpeed);
        m_cycler->SetFeeder(kFeederSpeed);
    }


    SmartDashboard::PutBoolean("TEST_READY_TO_FIRE", *m_turretready);
    SmartDashboard::PutBoolean("TEST_FIRING", m_timer.Get() != 0);
}

bool CyclerLaunch::IsFinished() {
    return m_timer.Get() > kTimeLaunch;
}

void CyclerLaunch::End(bool interrupted) {
    m_firing = false;
    *m_finished = true;
    m_cycler->SetFeeder(0);
    m_cycler->SetTurnTable(0);
}