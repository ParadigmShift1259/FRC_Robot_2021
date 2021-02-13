#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"

#include "Constants.h"

class HomeTarget : public frc2::CommandHelper<frc2::CommandBase, HomeTarget> {
 public:
  explicit HomeTarget(FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood, bool* turretready);

  void Execute() override;

 private:
  FlywheelSubsystem* m_flywheel;
  TurretSubsystem* m_turret;
  HoodSubsystem* m_hood;
  bool* m_turretready;
};