#include "commands/Fire.h"
#include "Constants.h"

Fire::Fire(FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood, 
            IntakeSubsystem* intake, CyclerSubsystem* cycler, VisionSubsystem* vision,
            bool* m_cyclerready)
: m_turretready(false)
, m_cyclerready(m_cyclerready)
, m_firing(false)
, m_finished(false)
{
  // All three run parallel, but the third has a delay hack through boolean pointers
  // Only CyclerPrepare has an end, as CyclerLaunch must run once the Cycler is in the right position
  AddCommands(
    // Home flywheel, turret, and hood to the correct speeds based on tuned fit function
    HomeTarget(flywheel, turret, hood, vision, &m_turretready, &m_firing, &m_finished),
    // If m_cycler ready and turret ready are true, cycler launch drives all of the balls through
    // m_turretready and m_cyclerready are checked every loop until success
    //CyclerFire(cycler, &m_turretready, m_cyclerready, &m_finished)
    CyclerLaunch(cycler, &m_turretready, m_cyclerready, &m_firing, &m_finished)
  );
}