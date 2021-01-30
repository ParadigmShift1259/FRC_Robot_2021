#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FlywheelSubsystem.h"

#include "Constants.h"

class FlywheelIdle : public frc2::CommandHelper<frc2::CommandBase, FlywheelIdle> {
 public:
  explicit FlywheelIdle(FlywheelSubsystem* subsystem);

  void Execute() override;

 private:
  FlywheelSubsystem* m_flywheel;
};