#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class FindClosestBall : public frc2::CommandHelper<frc2::CommandBase, FindClosestBall> {
 public:
  explicit FindClosestBall(DriveSubsystem* subsystem, bool* isRedPath);

  void Execute() override;

  bool IsFinished() override;

  void End(bool interrupted) override;

 private:
  DriveSubsystem* m_drive;
  bool m_bFoundBall = false;
  bool* m_isRedPath = nullptr;
};