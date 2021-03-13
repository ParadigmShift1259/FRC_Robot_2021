
#include "commands/IntakeIngest.h"
#include "Constants.h"

using namespace IntakeConstants;

IntakeIngest::IntakeIngest(IntakeSubsystem* subsystem, bool* m_cyclerready) 
: m_intake(subsystem)
, m_cyclerready(m_cyclerready)
{
  printf("Initailized Intake Ingest command");
  AddRequirements({subsystem});
}

void IntakeIngest::Execute() {
    printf("Running intake");
    *m_cyclerready = false;
    m_intake->Set(kIngestHigh);
}
