#include "commands/CyclerPrepare.h"
#include "Constants.h"

using namespace CyclerConstants;

CyclerPrepare::CyclerPrepare(CyclerSubsystem* subsystem, bool* cyclerready)
 : m_cycler(subsystem)
 , m_cyclerready(cyclerready)
{
    AddRequirements({subsystem});
}

void CyclerPrepare::Initialize() {
    m_cycler->StartDetection();
}

void CyclerPrepare::Execute() {
    m_cycler->SetTurnTable(kTurnTableHoneSpeed);
    m_cycler->SetFeeder(0);
}

bool CyclerPrepare::IsFinished() {
    return m_cycler->AtPosition();
}

void CyclerPrepare::End(bool interrupted) {
    if (!interrupted) {
        *m_cyclerready = true;
        m_cycler->ResetSensor();
        m_cycler->SetFeeder(0);
        m_cycler->SetTurnTable(0);
    }
}