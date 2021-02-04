#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/TurretSubsystem.h"

#include "Constants.h"

class TurretControl : public frc2::CommandHelper<frc2::CommandBase, TurretControl> {
 public:
  explicit TurretControl(TurretSubsystem* subsystem);

  void Execute() override;

 private:
  TurretSubsystem* m_turret;
};