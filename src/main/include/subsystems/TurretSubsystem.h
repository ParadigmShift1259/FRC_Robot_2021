
#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#include "Constants.h"
#include "common/Util.h"

using namespace std;
using namespace frc;

class TurretSubsystem : public frc2::SubsystemBase
{
public:
    TurretSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    // Most calculations currently depend on counter clockwise turning, with 0 as "front"

    /// Turns the turret to a specified angle on the turret
    /// \param angle        Angle desired to turn to, must be within the boundaries of the turret, must be positive, in degrees
    void TurnTo(double angle);

    /// Turns the turret based on the robot angle
    /// \param robotAngle        Robot angle to turn to, must be positive, in degrees
    void TurnToRobot(double robotAngle);

    /// Turns the turret based on an absolute field angle
    /// \param gyroAngle        Field angle to turn to, must be positive
    /// \param robotAngle       Current robot angle, must be positive, in degrees
    void TurnToField(double gyroAngle, double robotAngle);

    /// Turns the turret to an angle added to the current robot position
    /// \param angle        Angle that should be added to the robot position and turned to, can be either positive or negative, in degrees
    void TurnToRelative(double angle);

    /// Returns whether or not the turret is at the desired setpoint
    bool isAtSetpoint();

protected:
    /// Converts motor ticks into turret rotation, in degrees
    /// \param ticks        Number of ticks to be converted
    double TicksToDegrees(double ticks);

    /// Converts turret rotation in degrees into motor ticks
    /// \param degrees      Number of degrees to be converted
    double DegreesToTicks(double degrees);

private:    
    TalonSRX m_turretmotor;
    double m_currentAngle;
};
