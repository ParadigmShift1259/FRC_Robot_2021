#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class DriveToFinishLine : public frc2::CommandHelper<frc2::CommandBase, DriveToFinishLine> {
 public:
  explicit DriveToFinishLine(DriveSubsystem* subsystem);

  void Execute() override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  double m_xTarget = 0.0;
  double m_yTarget = 0.0;
};