#include "commands/CyclerPrepare.h"
#include "Constants.h"

using namespace CyclerConstants;

CyclerPrepare::CyclerPrepare(CyclerSubsystem* subsystem, bool* cyclerready)
 : m_cycler(subsystem)
 , m_cyclerready(cyclerready)
{
  AddRequirements({subsystem});
}

void CyclerPrepare::Execute() {
    // Sets the cycler to rotate until the paddle position is at the feeder position
    // m_cycler->SetPaddlePosition();
    if (m_cycler->TicksToDegrees(m_cycler->GetPosition())
    % 360 < 2 || m_cycler->TicksToDegrees(m_cycler->GetPosition())
    % 360 > -2) {
        m_cycler->SetTurnTable(CyclerConstants::kTurnTableSpeed);
        m_cycler->SetFeeder(0);
    }
    else {
        m_cycler->SetTurnTable(0);
        m_cycler->SetFeeder(CyclerConstants::kFeederSpeed);
    }

}

bool CyclerPrepare::IsFinished() {
    // m_cycler->AtSetpoint();
    return false;
}

void CyclerPrepare::End(bool interrupted) {
    if (!interrupted) {
        *m_cyclerready = true;
    }
}