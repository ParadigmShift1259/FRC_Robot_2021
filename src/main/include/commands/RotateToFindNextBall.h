#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class RotateToFindNextBall : public frc2::CommandHelper<frc2::CommandBase, RotateToFindNextBall> {
 public:
  explicit RotateToFindNextBall(DriveSubsystem* subsystem, bool *isRedPath);

  void Execute() override;

  bool IsFinished() override;

  void End(bool interrupted) override;

 private:
  DriveSubsystem* m_drive;
  bool* m_isRedPath = nullptr;
  bool m_headingSet = false;
  int m_ballCount = 1;
  double m_heading;
};