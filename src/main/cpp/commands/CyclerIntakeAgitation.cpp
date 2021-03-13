#include "commands/CyclerIntakeAgitation.h"
#include "Constants.h"

CyclerIntakeAgitation::CyclerIntakeAgitation(IntakeSubsystem* intake, CyclerSubsystem* cycler, bool* cyclerready)
{
  AddCommands(
    // Running intake
    IntakeIngest(intake, cyclerready),
    // Agitate Cycler
    CyclerAgitation(cycler)
  );
}