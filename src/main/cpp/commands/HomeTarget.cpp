#include "commands/HomeTarget.h"
#include "Constants.h"

HomeTarget::HomeTarget(FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood, bool* turretready)
 : m_flywheel(flywheel)
 , m_turret(turret)
 , m_hood(hood)
 , m_turretready(turretready)
{
  AddRequirements({flywheel, turret, hood});
}

void HomeTarget::Execute()
{
  // Homes flywheel, turret, and hood to the right angles through a formula
}