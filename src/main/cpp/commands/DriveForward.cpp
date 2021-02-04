
#include "commands/DriveForward.h"

DriveForward::DriveForward(DriveSubsystem* subsystem) : m_drive(subsystem) {
  AddRequirements({subsystem});
}

void DriveForward::Execute() {
    m_drive->Drive(units::meters_per_second_t(0.5),
                units::meters_per_second_t(0),
                units::radians_per_second_t(0),
                false);
}