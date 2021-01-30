
#pragma once

#include "commands/IntakeIngest.h"

IntakeIngest::IntakeIngest(IntakeSubsystem* subsystem) : m_intake(subsystem) {
  AddRequirements({subsystem});
}

void IntakeIngest::Execute() {
    m_intake->Run(IntakeConstants::kIngestHigh);
}