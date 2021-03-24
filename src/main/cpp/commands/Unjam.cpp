#include "commands/Unjam.h"
#include "Constants.h"

using namespace CyclerConstants;
using namespace IntakeConstants;

Unjam::Unjam(CyclerSubsystem* cycler, IntakeSubsystem* intake)
 : m_cycler(cycler)
 , m_intake(intake)
{
    AddRequirements({cycler, intake});
}

void Unjam::Execute() {
    m_cycler->SetTurnTable(-1.0 * kTurnTableHoneSpeed);
    m_cycler->SetFeeder(-1.0 * kFeederSpeed);
    m_intake->Set(kReleaseHigh);
}