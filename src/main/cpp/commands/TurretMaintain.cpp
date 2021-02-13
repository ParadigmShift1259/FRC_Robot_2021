
#include "commands/TurretMaintain.h"
#include "Constants.h"

using namespace TurretConstants;

TurretMaintain::TurretMaintain(TurretSubsystem* subsystem) : m_turret(subsystem) {
  AddRequirements({subsystem});
}

void TurretMaintain::Execute() {
    m_turret->TurnToField(TurretSubsystem::TicksToDegrees(TurretSubsystem::m_turretmotor.GetSelectedSensorPosition()), DriveSubsystem::GetHeadingAsRot2d()); 
}