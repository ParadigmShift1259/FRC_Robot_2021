
#include "commands/HoodRaise.h"
#include "Constants.h"

using namespace HoodConstants;

HoodRaise::HoodRaise(HoodSubsystem* subsystem) : m_hood(subsystem) {
  AddRequirements({subsystem});
}

void HoodRaise::Execute() {
    m_hood->Set(HoodConstants::kTestServoSpeed);
}