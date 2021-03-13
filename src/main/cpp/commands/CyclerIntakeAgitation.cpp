#include "commands/CyclerIntakeAgitation.h"
#include "Constants.h"

CyclerIntakeAgitation::CyclerIntakeAgitation(IntakeSubsystem* intake, CyclerSubsystem* cycler, double power)
{
  AddCommands(
    // Running intake
    IntakeIngest(intake, power),
    // Agitate Cycler
    CyclerAgitation(cycler)
  );
}