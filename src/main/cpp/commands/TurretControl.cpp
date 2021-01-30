
#include "commands/TurretControl.h"
#include "Constants.h"

using namespace TurretConstants;

TurretControl::TurretControl(DriveSubsystem* subsystem) : m_turret(subsystem) {
  AddRequirements({subsystem});
}

void TurretControl::Execute() {
    m_turret->TurnToRelative(TurretConstants::kAddedAngle); 
    // Needs to be modified later, I don't really understand the X and Y logic -- Kellen
}