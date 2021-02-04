#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/HoodSubsystem.h"

#include "Constants.h"

class HoodRaise : public frc2::CommandHelper<frc2::CommandBase, HoodRaise> {
 public:
  explicit HoodRaise(HoodSubsystem* subsystem);

  void Execute() override;

 private:
  HoodSubsystem* m_hood;
};