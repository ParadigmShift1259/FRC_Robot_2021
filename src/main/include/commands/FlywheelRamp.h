#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FlywheelSubsystem.h"

#include "Constants.h"

class FlywheelRamp : public frc2::CommandHelper<frc2::CommandBase, FlywheelRamp> {
 public:
  explicit FlywheelRamp(FlywheelSubsystem* subsystem);

  void Execute() override;

 private:
  FlywheelSubsystem* m_flywheel;
};