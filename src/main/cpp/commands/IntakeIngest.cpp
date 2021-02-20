
#include "commands/IntakeIngest.h"
#include "Constants.h"

using namespace IntakeConstants;

IntakeIngest::IntakeIngest(IntakeSubsystem* subsystem) : m_intake(subsystem) {
  AddRequirements({subsystem});
}

void IntakeIngest::Execute() {
    m_intake->Set(kIngestHigh);
}
