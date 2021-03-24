
#include "Constants.h"
#include "commands/DriveToBall.h"

DriveToBall::DriveToBall(DriveSubsystem* driveSubsystem, IntakeSubsystem* intakeSubsystem)
  : m_drive(driveSubsystem)
  , m_intake(intakeSubsystem)
{
  AddRequirements({driveSubsystem});
  AddRequirements({intakeSubsystem});
}

void DriveToBall::Execute()
{
    double dist = SmartDashboard::GetNumber("ZDistance", -1.0);
    if (!m_bAcceptVisionDist)
    {
      frc::Pose2d position = m_drive->GetPose();
      double deltaX = m_xTarget - position.X().to<double>();
      double deltaY = m_yTarget - position.Y().to<double>();
      dist = sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    if (m_bAcceptVisionDist && dist <= 25.0)
    {
      m_bAcceptVisionDist = false;
      m_intake->Set(IntakeConstants::kIngestHigh);
      m_heading = m_drive->GetHeading();
      double estimateAngle;
      // if (m_heading <= 90)
      // {
        estimateAngle = m_heading;
      // }
      // else if (m_heading <= 270)
      // {
      //   estimateAngle = fabs(180 - m_heading);
      // }
      // else
      // {
      //   estimateAngle = fabs(360 - m_heading);
      // }

      frc::Pose2d position = m_drive->GetPose();      
      m_xTarget = position.X().to<double>() + 35 * sin(estimateAngle);
      m_yTarget = position.Y().to<double>() + 35 * cos(estimateAngle);
    }
    else if (dist <= 1.0)
    {
      m_bFinished = true;
    }
    else
    {    
      double angle = SmartDashboard::GetNumber("XAngle", 180.0);
      printf("Angle Degree %.3f\n", angle);
      if (abs(angle) <= 5.0)
      {
        m_drive->Drive(units::meters_per_second_t(0.5),
            units::meters_per_second_t(0),
            units::radians_per_second_t(0),
            false);
      }
      // else if (angle != 180.0)
      // {
      //   units::degree_t angleDegree(-1.0 * angle);
      //   m_drive->Drive(units::meters_per_second_t(0.5),
      //               units::meters_per_second_t(0),
      //               units::radians_per_second_t(angleDegree / 2.0_s),
      //               false);      
      // }
    }
}

bool DriveToBall::IsFinished() {
    return m_bFinished;
}

void DriveToBall::End(bool interrupted) {
    m_bFinished = false;
    m_bAcceptVisionDist = true;
    m_intake->Set(0.0);
}