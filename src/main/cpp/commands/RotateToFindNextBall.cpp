
#include "commands/RotateToFindNextBall.h"

RotateToFindNextBall::RotateToFindNextBall(DriveSubsystem* subsystem, bool* isRedPath)
  : m_drive(subsystem)
  , m_isRedPath(isRedPath)
{
  AddRequirements({subsystem});
}

void RotateToFindNextBall::Execute()
{
  if (!m_headingSet)
  {
    if (*m_isRedPath)
    {
      if (m_ballCount == 1)
      {
        m_heading = -20.0 * wpi::math::pi / 180.0;
      }
      else if (m_ballCount == 2)
      {
        m_heading = 60.0 * wpi::math::pi / 180.0;
      }
    }
    else
    {
      if (m_ballCount == 1)
      {
        m_heading = 60.0 * wpi::math::pi / 180.0;
      }
      else if (m_ballCount == 2)
      {
        m_heading = -40.0 * wpi::math::pi / 180.0;
      }
    }
    printf("Heading %.3f m_ballCount %d\n", m_heading, m_ballCount);
    m_headingSet = true;
  }

  m_drive->RotationDrive(units::meters_per_second_t(0),
              units::meters_per_second_t(0),
              units::radian_t(m_heading),
              true);
}

bool RotateToFindNextBall::IsFinished()
{
  //double distNetTable = SmartDashboard::GetNumber("ZDistance", -1.0);

  double gyroHeadingRadians = m_drive->GetHeading() * wpi::math::pi / 180.0;
  printf("Gyro Heading %.3f, Target Heading %.3f, Diff %.3f\n", gyroHeadingRadians, m_heading, fabs(gyroHeadingRadians - m_heading));
  if (fabs(gyroHeadingRadians - m_heading) <= 2.0 * wpi::math::pi / 180.0)
  {
    return true;
  }
  return false;
}

void RotateToFindNextBall::End(bool interrupted) {
  m_headingSet = false;
  m_ballCount++;
  printf("RotateToFindNextBall Ended ball count %d\n", m_ballCount);
}