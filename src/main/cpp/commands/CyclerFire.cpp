#include "commands/CyclerFire.h"
#include "Constants.h"

CyclerFire::CyclerFire(CyclerSubsystem* cycler, 
                        bool* turretready, bool* firing, bool* finished, 
                        double launchtime)
{
    AddCommands(
        CyclerPrepare(cycler, false),
        CyclerLaunch(cycler, turretready, firing, finished, launchtime),
        CyclerPrepare(cycler, true)
    );
}