#include "commands/Fire.h"
#include "Constants.h"

Fire::Fire(FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood, 
            IntakeSubsystem* intake, CyclerSubsystem* cycler)
: m_turretready(false)
, m_cyclerready(false)
{
  // All three run parallel, but the third has a delay hack through boolean pointers
  // Only CyclerPrepare has an end, as CyclerLaunch must run once the Cycler is in the right position
  AddCommands(
    // Home flywheel, turret, and hood to the correct speeds based on tuned fit function
    HomeTarget(flywheel, turret, hood, &m_turretready),
    // Prepare cycler by directing to the right position
    CyclerPrepare(cycler, &m_cyclerready),
    // If m_cycler ready and turret ready are true, cycler launch drives all of the balls through
    // m_turretready and m_cyclerready are checked every loop until success
    CyclerLaunch(cycler, &m_turretready, &m_cyclerready)
  );
}