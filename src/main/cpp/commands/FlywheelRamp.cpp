#include "commands/FlywheelRamp.h"
#include "Constants.h"

using namespace FlywheelConstants;

FlywheelRamp::FlywheelRamp(FlywheelSubsystem* subsystem) : m_flywheel(subsystem) {
  AddRequirements({subsystem});
}

void FlywheelRamp::Execute() {
    m_flywheel->SetRPM(FlywheelConstants::kRampRPM);
}