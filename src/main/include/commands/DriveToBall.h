#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class DriveToBall : public frc2::CommandHelper<frc2::CommandBase, DriveToBall> {
 public:
  explicit DriveToBall(DriveSubsystem* driveSubsystem, IntakeSubsystem* intakeSubsystem, Gyro* gyro);

  void Execute() override;

  bool IsFinished() override;

  void End(bool interrupted) override;

 private:
  DriveSubsystem* m_drive;
  IntakeSubsystem* m_intake;
  double m_heading;
  double m_xTarget;
  double m_yTarget;
  bool m_bAcceptVisionDist = true;
  bool m_bFinished = false;
  Gyro* m_gyro;
};