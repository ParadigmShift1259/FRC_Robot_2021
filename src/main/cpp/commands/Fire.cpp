#include "commands/Fire.h"
#include "Constants.h"

Fire::Fire(frc::XboxController* controller, FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood, 
            IntakeSubsystem* intake, CyclerSubsystem* cycler, VisionSubsystem* vision,
            bool* turretready, bool* firing, bool* finished,
            double launchtime)
: m_controller(controller)
, m_turretready(turretready)
, m_firing(firing)
, m_finished(finished)
{
  // All three run parallel, but the third has a delay hack through boolean pointers
  // Only CyclerPrepare has an end, as CyclerLaunch must run once the Cycler is in the right position
  AddCommands(
    // Home flywheel, turret, and hood to the correct speeds based on tuned fit function
    HomeTarget(m_controller, flywheel, turret, hood, vision, m_turretready, m_firing, m_finished),
    // If m_cycler ready and turret ready are true, cycler launch drives all of the balls through
    // m_turretready is checked every loop until success
    CyclerFire(cycler, m_turretready, m_firing, m_finished, launchtime)
  );
}