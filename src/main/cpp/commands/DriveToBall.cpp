
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
    double dist = SmartDashboard::GetNumber("ZDistance", -1.0) / 39.37;
    m_heading = m_drive->GetHeading() * wpi::math::pi / 180.0;

    if (!m_bAcceptVisionDist)
    {
      frc::Pose2d position = m_drive->GetPose();
      double deltaX = m_xTarget - position.X().to<double>();
      double deltaY = m_yTarget - position.Y().to<double>();
      dist = sqrt(deltaX * deltaX + deltaY * deltaY);
      printf("Calculated Distanse %.3f, deltaX, %.3f, deltaY, %.3f\n", dist, deltaX, deltaY);
    }

    if (m_bAcceptVisionDist && dist <= 0.762) // 30 in
    {
      printf("Starting intake\n");
      m_intake->Set(IntakeConstants::kIngestHigh);
    }

    if (m_bAcceptVisionDist && dist <= 0.635) // 25 in
    {
      m_bAcceptVisionDist = false;
      printf("Ball too close\n");
      m_intake->Set(IntakeConstants::kIngestHigh);
      frc::Pose2d position = m_drive->GetPose();      
      m_xTarget = position.X().to<double>() + 0.889 * cos(m_heading); //0.889 meters equals 35 inches
      m_yTarget = position.Y().to<double>() + 0.889 * sin(m_heading);
    }
    else if (dist <= 0.2) //0.2 meters
    {
      m_bFinished = true;
    }
    else
    {    
      double angle = SmartDashboard::GetNumber("XAngle", 180.0);
      units::degree_t angleDegree(m_heading - angle);
      m_drive->RotationDrive(units::meters_per_second_t(cos(m_heading)),
                  units::meters_per_second_t(sin(m_heading)),
                  units::radian_t(angleDegree),
                  true);      
    }
}

bool DriveToBall::IsFinished() {
    return m_bFinished;
}

void DriveToBall::End(bool interrupted) {
    m_bFinished = false;
    m_bAcceptVisionDist = true;
    m_intake->Set(0.0);
    m_drive->Drive(units::meters_per_second_t(0.0),
                units::meters_per_second_t(0.0),
                units::radians_per_second_t(0.0),
                false);      
}