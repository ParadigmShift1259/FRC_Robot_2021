#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/CyclerSubsystem.h"

#include "commands/CyclerAgitation.h"
#include "commands/IntakeIngest.h"

#include "Constants.h"

class CyclerIntakeAgitation : public frc2::CommandHelper<frc2::ParallelCommandGroup, CyclerIntakeAgitation> {
public:
    CyclerIntakeAgitation(IntakeSubsystem* intake, CyclerSubsystem* cycler, double speed);
};