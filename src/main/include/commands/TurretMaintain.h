#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/TurretSubsystem.h"

#include "Constants.h"

class TurretMaintain : public frc2::CommandHelper<frc2::CommandBase, TurretMaintain> {
 public:
  explicit TurretMaintain(TurretSubsystem* subsystem);

  void Execute() override;

 private:
  DriveSubsystem* m_drive;
  TurretSubsystem* m_turret;
};