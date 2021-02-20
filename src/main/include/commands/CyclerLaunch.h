#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CyclerSubsystem.h"

#include "Constants.h"

class CyclerLaunch : public frc2::CommandHelper<frc2::CommandBase, CyclerLaunch> {
public:
    explicit CyclerLaunch(CyclerSubsystem* subsystem, bool* turretready, bool* cyclerready);

    void Execute() override;

 private:
    CyclerSubsystem* m_cycler;
    bool* m_turretready;
    bool* m_cyclerready;
};