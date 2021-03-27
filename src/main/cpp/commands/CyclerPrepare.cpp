#include "commands/CyclerPrepare.h"
#include "Constants.h"

using namespace CyclerConstants;

CyclerPrepare::CyclerPrepare(CyclerSubsystem* subsystem, bool reset)
 : m_cycler(subsystem)
{
    AddRequirements({subsystem});
    if (reset)
        m_cycler->ResetSensor();
}

void CyclerPrepare::Initialize() {
    m_cycler->StartDetection();
}

void CyclerPrepare::Execute() {
    printf("Cycler Preparing...\n");
    m_cycler->SetTurnTable(kTurnTableHoneSpeed);
    m_cycler->SetFeeder(0);
}

bool CyclerPrepare::IsFinished() {
    printf("Cycler Check end\n");
    return m_cycler->AtPosition();
}

void CyclerPrepare::End(bool interrupted) {
    printf("Cycler End\n");
    if (interrupted) {
        m_cycler->ResetSensor();
    }
    else {
        printf("Cycler Ready\n");
    }
    m_cycler->SetFeeder(0);
    m_cycler->SetTurnTable(0);

}