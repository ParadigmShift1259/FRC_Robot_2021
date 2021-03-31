
#include "Constants.h"
#include "commands/DriveToBall.h"

DriveToBall::DriveToBall(DriveSubsystem* driveSubsystem, IntakeSubsystem* intakeSubsystem)
  : m_drive(driveSubsystem)
  , m_intake(intakeSubsystem)
{
  AddRequirements({driveSubsystem});
  AddRequirements({intakeSubsystem});
  printf("Starting intake\n");
  m_intake->Set(IntakeConstants::kIngestHigh);
}

void DriveToBall::Execute()
{
    double distNetTable = SmartDashboard::GetNumber("ZDistance", -1.0);
    double distMeters =  distNetTable / 39.37;
    m_heading = m_drive->GetHeading() * wpi::math::pi / 180.0;

    if (distMeters <= 0.762 || !m_bAcceptVisionDist) // 30 inches and also dist == -1.0
    {
      m_bAcceptVisionDist = false;
      frc::Pose2d position = m_drive->GetPose();
      double deltaX = m_xTarget - position.X().to<double>();
      double deltaY = m_yTarget - position.Y().to<double>();
      double distEstimated = sqrt(deltaX * deltaX + deltaY * deltaY);
      static int count = 0;
      if (count++ % 10 == 0)
      {
        printf("Ball too close: Calculated Distance %.3f, Vision Dist %.3f Vision Dist NetTable %.3f deltaX, %.3f, deltaY %.3f m_desiredHeading %.3f\n", distEstimated, distMeters, distNetTable, deltaX, deltaY, m_desiredHeading);   
      }
      if (distEstimated <= 0.3048) //12 inches
      {
        printf("Ball too close: Calculated Distance %.3f, Vision Dist %.3f Vision Dist NetTable %.3f deltaX, %.3f, deltaY %.3f finished\n", distEstimated, distMeters, distNetTable, deltaX, deltaY);   
        m_bFinished = true;
      }

      // units::radian_t desiredAngle(m_heading - m_desiredHeading);
      // m_drive->RotationDrive(units::meters_per_second_t(0.25),
      //             units::meters_per_second_t(0.0),
      //             desiredAngle,
      //             false);      
      m_drive->Drive(units::meters_per_second_t(0.25),
                  units::meters_per_second_t(0.0),
                  units::radians_per_second_t(0.0),
                  false);      
    }
    else
    {
      m_desiredHeading = SmartDashboard::GetNumber("XAngle", 180.0) * wpi::math::pi / 180.0;
      //units::radian_t desiredAngle(m_heading - m_desiredHeading);
      units::radian_t desiredAngle(-1.0 * m_desiredHeading);
      units::radians_per_second_t desiredRate(desiredAngle / 0.05_s);

      // m_drive->RotationDrive(units::meters_per_second_t(0.25),
      //             units::meters_per_second_t(0.0),
      //             desiredAngle,
      //             false);      
      m_drive->Drive(units::meters_per_second_t(0.25),
                  units::meters_per_second_t(0.0),
                  //units::radians_per_second_t(0.0),
                  desiredRate,
                  false);      

      if (!m_targetPointSet)
      {
        m_targetPointSet = true;
        frc::Pose2d position = m_drive->GetPose();      
        m_xTarget = position.X().to<double>() + (distMeters + 0.254) * cos(m_heading); //The target is 10 inches beyond the distance found by the pi
        m_yTarget = position.Y().to<double>() - (distMeters + 0.254) * sin(-1.0 * m_heading);
        printf("xTarget %.3f, yTarget %.3f, heading %.3f, piAngle %.3f, dist %.3f\n", m_xTarget, m_yTarget, m_heading, m_desiredHeading, distMeters);
      }
      // frc::Pose2d position = m_drive->GetPose();      
      // double xTarget = position.X().to<double>() + (distMeters + 0.254) * cos(m_heading); //The target is 10 inches beyond the distance found by the pi
      // double yTarget = position.Y().to<double>() - (distMeters + 0.254) * sin(-1.0 * m_heading);
      // printf("check xTarget %.3f, yTarget %.3f, heading %.3f, piAngle %.3f, dist %.3f\n", xTarget, yTarget, m_heading, m_desiredHeading, distMeters);
    }
}

bool DriveToBall::IsFinished() {
    return m_bFinished;
}

void DriveToBall::End(bool interrupted) {
    m_bFinished = false;
    m_bAcceptVisionDist = true;
    m_targetPointSet = false;
    // m_intake->Set(0.0);
    m_drive->Drive(units::meters_per_second_t(0.0),
                units::meters_per_second_t(0.0),
                units::radians_per_second_t(0.0),
                false);
    printf("DriveToBall Ended\n");
}