
#include "commands/IntakeRelease.h"
#include "Constants.h"

using namespace IntakeConstants;

IntakeRelease::IntakeRelease(IntakeSubsystem* subsystem) : m_intake(subsystem) {
  AddRequirements({subsystem});
}

void IntakeRelease::Execute() {
  m_intake->Set(kReleaseHigh);
}

void IntakeRelease::End(bool interrupted) {
  m_intake->Set(0);
}
