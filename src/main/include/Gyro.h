#pragma once

#include "Constants.h"
#include "units/angle.h"
#include <frc/geometry/Rotation2d.h>

#include <ctre/phoenix.h>

using namespace DriveConstants;
using namespace units;

class Gyro
{
public:

    Gyro();

    /// Returns the heading of the robot.
    /// \return the robot's heading in degrees, from -180 to 180
    double GetHeading();
    frc::Rotation2d GetHeadingAsRot2d() { return frc::Rotation2d(degree_t(GetHeading())); }

    /// Zeroes the heading of the robot.
    void ZeroHeading();

    /// Returns the turn rate of the robot.
    /// \return The turn rate of the robot, in degrees per second
    double GetTurnRate();

    PigeonIMU m_gyro;
};
