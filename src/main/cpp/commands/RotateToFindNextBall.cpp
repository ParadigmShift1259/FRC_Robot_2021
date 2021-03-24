
#include "commands/RotateToFindNextBall.h"

RotateToFindNextBall::RotateToFindNextBall(DriveSubsystem* subsystem, bool isRedPath)
  : m_drive(subsystem)
  , m_isRedPath(isRedPath)
{
  AddRequirements({subsystem});
  m_timer.Reset();
}

void RotateToFindNextBall::Execute() {
    double time = m_timer.Get();

    if (time == 0.0)
    {
      if (m_isRedPath)
      {
        m_drive->Drive(units::meters_per_second_t(0),
                    units::meters_per_second_t(0),
                    units::radians_per_second_t(units::degree_t(40.0) / 1.0_s),
                    false);
        m_timer.Start();
      }
    }
    else if (time >= 1.0)
    {
      m_bFinished = true;
    }
}

bool RotateToFindNextBall::IsFinished() {
  return m_bFinished;
}

void RotateToFindNextBall::End(bool interrupted) {
  m_bFinished = false;
  m_timer.Stop();
  m_timer.Reset();
}