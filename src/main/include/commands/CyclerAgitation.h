#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>


#include "subsystems/CyclerSubsystem.h"

#include "Constants.h"

class CyclerAgitation : public frc2::CommandHelper<frc2::CommandBase, CyclerAgitation> {
 public:
  explicit CyclerAgitation(CyclerSubsystem* subsystem);

  void Execute() override;

  void Initialize() override;

 private:
  CyclerSubsystem* m_turret;
};