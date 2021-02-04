#include "commands/FlywheelIdle.h"
#include "Constants.h"

using namespace FlywheelConstants;

FlywheelIdle::FlywheelIdle(FlywheelSubsystem* subsystem) : m_flywheel(subsystem) {
  AddRequirements({subsystem});
}

void FlywheelIdle::Execute() {
    m_flywheel->SetRPM(FlywheelConstants::kIdleRPM);
}