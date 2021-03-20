
#include "commands/VisionDetect.h"
#include "Constants.h"

using namespace VisionConstants;

VisionDetect::VisionDetect(VisionSubsystem* subsystem) : m_vision(subsystem) {
  AddRequirements({subsystem});
}

void VisionDetect::Execute() {
    m_vision->GetDirection();
}
