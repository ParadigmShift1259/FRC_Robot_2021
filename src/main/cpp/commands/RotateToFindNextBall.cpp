
#include "commands/RotateToFindNextBall.h"

RotateToFindNextBall::RotateToFindNextBall(DriveSubsystem* subsystem, bool isRedPath)
  : m_drive(subsystem)
  , m_isRedPath(isRedPath)
{
  AddRequirements({subsystem});
  if (m_isRedPath)
  {
    m_heading = (m_drive->GetHeading() + 40.0) * wpi::math::pi / 180.0;
  }
  else
  {
    m_heading = (m_drive->GetHeading() - 40.0)  * wpi::math::pi / 180.0;
  }
}

void RotateToFindNextBall::Execute() {
  m_drive->RotationDrive(units::meters_per_second_t(0),
              units::meters_per_second_t(0),
              units::radian_t(m_heading),
              true);
}

bool RotateToFindNextBall::IsFinished() {
  if (fabs(m_drive->GetHeading() - m_heading) <= 2.0)
  {
    return true;
  }
  return false;
}

void RotateToFindNextBall::End(bool interrupted) {

}