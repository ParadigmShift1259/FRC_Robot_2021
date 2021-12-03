#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

class IntakeRelease : public frc2::CommandHelper<frc2::CommandBase, IntakeRelease> {
 public:
  explicit IntakeRelease(IntakeSubsystem* subsystem);

  void Execute() override;
  void End(bool interrupted) override;

 private:
  IntakeSubsystem* m_intake;
};