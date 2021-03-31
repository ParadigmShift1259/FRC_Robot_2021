
#include "commands/DriveToFinishLine.h"

DriveToFinishLine::DriveToFinishLine(DriveSubsystem* subsystem) : m_drive(subsystem) 
{
  AddRequirements({subsystem});
  frc::Pose2d position = m_drive->GetPose();
  m_xTarget = 8.763; //345.0 inches
  m_yTarget = position.Y().to<double>();
}

void DriveToFinishLine::Execute() {
    m_drive->RotationDrive(units::meters_per_second_t(1.0),
                units::meters_per_second_t(0.0),
                units::radian_t(0.0),
                true);
}

bool DriveToFinishLine::IsFinished() {
    frc::Pose2d position = m_drive->GetPose();
    double deltaX = m_xTarget - position.X().to<double>();
    double deltaY = m_yTarget - position.Y().to<double>();
    double dist = sqrt(deltaX * deltaX + deltaY * deltaY);

    if (dist <= 0.1)
    {
      return true; 
    }
    return false;
}