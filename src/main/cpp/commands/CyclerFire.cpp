#include "commands/CyclerFire.h"
#include "Constants.h"

CyclerFire::CyclerFire(CyclerSubsystem* cycler, bool* turretready, bool* firing, bool* finished)
{
    AddCommands(
        CyclerPrepare(cycler, false),
        CyclerLaunch(cycler, turretready, firing, finished)
    );
}