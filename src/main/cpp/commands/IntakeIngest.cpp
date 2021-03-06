
#include "commands/IntakeIngest.h"
#include "Constants.h"

using namespace IntakeConstants;

IntakeIngest::IntakeIngest(IntakeSubsystem* subsystem) 
: m_intake(subsystem)
{
  // printf("Initailized Intake Ingest command");
  AddRequirements({subsystem});
}

void IntakeIngest::Execute() {
    m_intake->Set(kIngestHigh);
}

void IntakeIngest::End(bool interrupted) {
    m_intake->Set(0);
}