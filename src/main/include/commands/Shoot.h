/*
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/TurretSubsystem.h"

#include "Constants.h"

class Shoot : public frc2::CommandHelper<frc2::CommandBase, Shoot> {
 public:
  explicit Shoot(TurretSubsystem* subsystem);

  void Execute() override;

 private:
  FlywheelSubsystem* m_flywheel;
  HoodSubsystem* m_hood;
  TurretSubsystem* m_turret;
};
*/