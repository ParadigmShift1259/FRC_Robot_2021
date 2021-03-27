#include "commands/CyclerLaunch.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CyclerConstants;

CyclerLaunch::CyclerLaunch(CyclerSubsystem* subsystem, 
                            bool* turretready, 
                            bool* firing, bool* finished)
 : m_cycler(subsystem)
 , m_turretready(turretready)
 , m_firing(firing)
 , m_finished(finished)
{
  AddRequirements({subsystem});
  *m_firing = false;
}

void CyclerLaunch::Initialize()
{
    m_timer.Reset();
    *m_firing = false;
}

void CyclerLaunch::Execute()
{
    printf("Cycler Launch running...\n");
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


    SmartDashboard::PutBoolean("TEST_READY_TO_FIRE", *m_turretready);
    SmartDashboard::PutBoolean("TEST_FIRING", *m_firing);
}

bool CyclerLaunch::IsFinished() {
    return m_timer.Get() > kTimeLaunch;
}

void CyclerLaunch::End(bool interrupted) {
    printf("CyclerLaunch Ended\n");
    if (interrupted) {
        printf("CyclerLaunch Interrupted\n");
    }
    else {
        *m_finished = true;
    }

    *m_firing = false;
    m_timer.Stop();
    m_cycler->ResetSensor();
    m_cycler->SetFeeder(0);
    m_cycler->SetTurnTable(0);

    SmartDashboard::PutBoolean("TEST_FIRING", *m_firing);
}