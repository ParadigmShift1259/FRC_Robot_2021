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
    m_cycler->SetTurnTable(kTurnTableHoneSpeed);
    m_cycler->SetFeeder(0);
}

// Set to true immediately because sensor was removed
bool CyclerPrepare::IsFinished() {
    return true; //m_cycler->AtPosition();
}

void CyclerPrepare::End(bool interrupted) {
    if (interrupted) {
        m_cycler->ResetSensor();
    }
    m_cycler->SetFeeder(0);
    m_cycler->SetTurnTable(0);

}