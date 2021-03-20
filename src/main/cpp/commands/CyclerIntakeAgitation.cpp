#include "commands/CyclerIntakeAgitation.h"
#include "Constants.h"

CyclerIntakeAgitation::CyclerIntakeAgitation(IntakeSubsystem* intake, CyclerSubsystem* cycler)
{
  AddCommands(
    // Running intake
    IntakeIngest(intake),
    // Agitate Cycler
    CyclerAgitation(cycler)
  );
}