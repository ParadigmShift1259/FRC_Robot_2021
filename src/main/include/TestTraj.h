#include <vector>
#include <frc/geometry/Translation2d.h>
#include <units/units.h>

#define RADIUS 10.0_ft

std::vector<frc::Pose2d> TestTrajCircle1 =
{
  frc::Pose2d(0.0*RADIUS, -1.0*RADIUS, 90.0_deg),
  frc::Pose2d(-0.5*RADIUS, -sqrt(3)/2.*RADIUS, 90.0_deg),
  frc::Pose2d(-sqrt(3)/2.*RADIUS, -0.5*RADIUS, 90.0_deg),
  frc::Pose2d(-1.0*RADIUS, 0.0*RADIUS, 90_deg),
  frc::Pose2d(-sqrt(3)/2.*RADIUS, 0.5*RADIUS, 90_deg),
  frc::Pose2d(-0.5*RADIUS, sqrt(3)/2.*RADIUS, 90_deg),
  frc::Pose2d(0.0*RADIUS, 1.0*RADIUS, 90_deg),
  frc::Pose2d(0.5*RADIUS, sqrt(3)/2.*RADIUS, 90_deg),
  frc::Pose2d(sqrt(3)/2.*RADIUS, 0.5*RADIUS, 90_deg),
  frc::Pose2d(1.0*RADIUS, 0.0*RADIUS, 90_deg),
  frc::Pose2d(sqrt(3)/2.*RADIUS, -0.5*RADIUS, 90_deg),
  frc::Pose2d(0.5*RADIUS, -sqrt(3)/2.*RADIUS, 90_deg),
};

std::vector<frc::Pose2d> TestTrajCircle2 =
{
  frc::Pose2d(0.0*RADIUS, 1.0*RADIUS, 0.0_deg),
  frc::Pose2d(0.5*RADIUS, sqrt(3)/2.*RADIUS, 330.0_deg),
  frc::Pose2d(sqrt(3)/2.*RADIUS, 0.5*RADIUS, 300.0_deg),
  frc::Pose2d(1.0*RADIUS, 0.0*RADIUS, 270.0_deg),
  frc::Pose2d(sqrt(3)/2.*RADIUS, -0.5*RADIUS, 240.0_deg),
  frc::Pose2d(0.5*RADIUS, -sqrt(3)/2.*RADIUS, 210.0_deg),
  frc::Pose2d(0.0*RADIUS, -1.0*RADIUS, 180.0_deg),
  frc::Pose2d(-0.5*RADIUS, -sqrt(3)/2.*RADIUS, 150.0_deg),
  frc::Pose2d(-sqrt(3)/2.*RADIUS, -0.5*RADIUS, 120.0_deg),
  frc::Pose2d(-1.0*RADIUS, 0.0*RADIUS, 90.0_deg),
  frc::Pose2d(-sqrt(3)/2.*RADIUS, 0.5*RADIUS, 60.0_deg),
  frc::Pose2d(-0.5*RADIUS, sqrt(3)/2.*RADIUS, 30.0_deg),
  frc::Pose2d(0.0*RADIUS, 1.0*RADIUS, 0.0_deg),
 };

std::vector<frc::Pose2d> TestTrajLine =
{
  frc::Pose2d(0_m, 0_m, 0.0_deg),
  frc::Pose2d(3_m, 0_m, 0.0_deg),
  frc::Pose2d(6_m, 0_m, 0.0_deg),
  frc::Pose2d(9_m, 0_m, 0.0_deg)
};
