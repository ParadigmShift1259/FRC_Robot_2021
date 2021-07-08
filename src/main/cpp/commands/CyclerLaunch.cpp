#include "commands/CyclerLaunch.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CyclerConstants;

CyclerLaunch::CyclerLaunch(CyclerSubsystem* subsystem, 
                            bool* turretready, bool* firing, bool* finished,
                            double launchtime)
 : m_cycler(subsystem)
 , m_turretready(turretready)
 , m_firing(firing)
 , m_finished(finished)
 , m_launchtime(launchtime)
{
  AddRequirements({subsystem});
  *m_turretready = false;
  *m_firing = false;
  *m_finished = false;
}

void CyclerLaunch::Initialize()
{
    m_timer.Reset();
    m_timer.Stop();
    *m_turretready = false;
    *m_firing = false;
    *m_finished = false;
}

void CyclerLaunch::Execute()
{
    if (*m_turretready)
    {
        *m_firing = true;
        m_timer.Start();
    }

    if (*m_firing)
    {
        m_cycler->SetTurnTable(kTurnTableSpeed);
        m_cycler->SetFeeder(kFeederSpeed);
    }
    else
    {
        m_cycler->SetTurnTable(0);
        m_cycler->SetFeeder(0);
    }


    // SmartDashboard::PutBoolean("TEST_READY_TO_FIRE", *m_turretready);
    // SmartDashboard::PutBoolean("TEST_FIRING", *m_firing);
}

bool CyclerLaunch::IsFinished() {
    return m_timer.Get() > m_launchtime;
}

void CyclerLaunch::End(bool interrupted) {
    *m_finished = true;
    *m_firing = false;
    m_timer.Stop();
    m_cycler->ResetSensor();
    m_cycler->SetFeeder(0);
    m_cycler->SetTurnTable(0);

    // SmartDashboard::PutBoolean("TEST_FIRING", *m_firing);
}