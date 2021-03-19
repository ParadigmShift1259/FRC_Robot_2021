#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CyclerSubsystem.h"

#include "Constants.h"

class CyclerPrepare : public frc2::CommandHelper<frc2::CommandBase, CyclerPrepare> {
public:
    explicit CyclerPrepare(CyclerSubsystem* subsystem, bool reset);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

 private:
    CyclerSubsystem* m_cycler;
    bool reset;
};