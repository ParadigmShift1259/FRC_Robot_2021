#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/CyclerSubsystem.h"

#include "commands/CyclerPrepare.h"

#include "Constants.h"

class CyclerLaunch : public frc2::CommandHelper<frc2::CommandBase, CyclerLaunch> {
public:
    explicit CyclerLaunch(CyclerSubsystem* subsystem, 
                            bool* turretready, bool* cyclerready, 
                            bool* firing, bool* finished);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;


 private:
    CyclerSubsystem* m_cycler;
    Timer m_timer;
    bool* m_turretready;
    bool* m_cyclerready;
    bool* m_firing;
    bool* m_finished;
};