
#include "commands/FindClosestBall.h"

FindClosestBall::FindClosestBall(DriveSubsystem* subsystem, bool* isRedPath)
  : m_drive(subsystem)
  , m_isRedPath(isRedPath)
{
  AddRequirements({subsystem});
}

void FindClosestBall::Execute() {
    static bool bPathKnown = false;
    double dist = SmartDashboard::GetNumber("ZDistance", -1.0);
    if (dist != -1.0)
    {
      if (bPathKnown == false)
      {
        if (dist < 100.0)
        {
          *m_isRedPath = true;
          bPathKnown = true;
          printf("Red Path Found\n");
        }
        else
        {
          *m_isRedPath = false;
          bPathKnown = true;
          printf("Blue Path Found\n");
        }
      }
      m_bFoundBall = true;
      printf("Ball Found\n");
    }
    else
    {
      if (bPathKnown == false)
      {
        *m_isRedPath = false;
        bPathKnown = true;
        printf("Blue Path Found\n");
      } 
      
      m_drive->Drive(units::meters_per_second_t(0.5),
                  units::meters_per_second_t(0.25),
                  units::radians_per_second_t(0),
                  false);      
    }
}

bool FindClosestBall::IsFinished() {
    return m_bFoundBall;
}

void FindClosestBall::End(bool interrupted) {
    m_bFoundBall = false;
}