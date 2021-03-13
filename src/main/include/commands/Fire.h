#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/CyclerSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#include "commands/CyclerLaunch.h"
#include "commands/CyclerPrepare.h"
#include "commands/HomeTarget.h"

#include "Constants.h"

class Fire : public frc2::CommandHelper<frc2::ParallelCommandGroup, Fire> {
public:
    Fire(   FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood,
            IntakeSubsystem* intake, CyclerSubsystem* cycler, VisionSubsystem* vision,
            bool* m_cyclerready);
private:
    bool m_turretready;
    bool* m_cyclerready;
};