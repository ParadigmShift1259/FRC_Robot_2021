#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"

class RotateToFindNextBall : public frc2::CommandHelper<frc2::CommandBase, RotateToFindNextBall> {
 public:
  explicit RotateToFindNextBall(DriveSubsystem* subsystem, bool isRedPath);

  void Execute() override;

  bool IsFinished() override;

  void End(bool interrupted) override;

 private:
  DriveSubsystem* m_drive;
  bool m_isRedPath = false;
  int m_callCount = 0;
  frc::Timer m_timer;
  bool m_bFinished = false;
};