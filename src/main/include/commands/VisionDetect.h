#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include "subsystems/VisionSubsystem.h"
#include "Constants.h"

class VisionDetect : public frc2::CommandHelper<frc2::CommandBase, VisionDetect> {
 public:
  explicit VisionDetect(VisionSubsystem* subsystem);

  void Execute() override;

 private:
  VisionSubsystem* m_vision;
  VisionSubsystem::BallDirection direction;

};