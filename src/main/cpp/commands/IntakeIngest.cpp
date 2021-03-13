
#include "commands/IntakeIngest.h"
#include "Constants.h"

using namespace IntakeConstants;

IntakeIngest::IntakeIngest(IntakeSubsystem* subsystem, double power) 
: m_intake(subsystem)
, power(power)
{
  printf("Initailized Intake Ingest command");
  AddRequirements({subsystem});
}

void IntakeIngest::Execute() {
    printf("Running intake");
    m_intake->Set(kIngestHigh);
}
