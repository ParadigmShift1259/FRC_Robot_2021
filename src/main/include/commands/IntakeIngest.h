#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

class IntakeIngest : public frc2::CommandHelper<frc2::CommandBase, IntakeIngest> {
 public:
  explicit IntakeIngest(IntakeSubsystem* subsystem, bool* m_cyclerready);

  void Execute() override;
  
  bool* m_cyclerready;

 private:
  IntakeSubsystem* m_intake;
};