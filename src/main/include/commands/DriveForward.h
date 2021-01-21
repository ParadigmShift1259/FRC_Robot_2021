#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class DriveForward : public frc2::CommandHelper<frc2::CommandBase, DriveForward> {
 public:
  explicit DriveForward(DriveSubsystem* subsystem);

  void Execute() override;

 private:
  DriveSubsystem* m_drive;
};