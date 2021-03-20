#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CyclerSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

class Unjam : public frc2::CommandHelper<frc2::CommandBase, Unjam> {
public:
    explicit Unjam(CyclerSubsystem* cycler, IntakeSubsystem* intake);

    void Execute() override;

 private:
    CyclerSubsystem* m_cycler;
    IntakeSubsystem* m_intake;
};