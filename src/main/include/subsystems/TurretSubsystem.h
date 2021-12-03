
#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#include "Constants.h"
#include "common/Util.h"
#include "Gyro.h"

using namespace std;
using namespace frc;

class TurretSubsystem : public frc2::SubsystemBase
{
public:
    TurretSubsystem(Gyro *gyro);

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Turns the turret to a specified angle on the turret
    /// Most calculations currently depend on counter clockwise turning, with 0 as "front"
    /// \param angle        Angle desired to turn to, must be within the boundaries of the turret, must be positive, in degrees
    void TurnTo(double angle, double minAngle=TurretConstants::kMinAngle, double maxAngle=TurretConstants::kMaxAngle);

    /// Turns the turret based on the robot angle
    /// \param robotAngle        Robot angle to turn to, must be positive, in degrees
    void TurnToRobot(double robotAngle);

    /// Turns the turret based on an absolute field angle
    /// \param deisredAngle        Field angle to turn to, must be positive
    void TurnToField(double desiredAngle);

    /// Turns the turret to an angle added to the current robot position
    /// \param angle        Angle that should be added to the robot position and turned to, can be either positive or negative, in degrees
    void TurnToRelative(double angle, double minAngle=TurretConstants::kMinAngle, double maxAngle=TurretConstants::kMaxAngle);

    /// Returns whether or not the turret is at the desired setpoint
    bool isAtSetpoint();

    /// Resets the turret back to the initial default position
    void ResetPosition();

    /// Set new PID Values for the turret from on the fly SmartDashboard values
    void SetNewPIDValues();

protected:
    /// Converts motor ticks into turret rotation, in degrees
    /// \param ticks        Number of ticks to be converted
    double TicksToDegrees(double ticks);

    /// Converts turret rotation in degrees into motor ticks
    /// \param degrees      Number of degrees to be converted
    double DegreesToTicks(double degrees);

private:
    /// 775 that rurns the shooting mechanism
    TalonSRX m_turretmotor;
    /// The current angle of the turret, in degrees
    double m_currentAngle;
    /// Gyro to determine field relative angles, from @ref RobotContainer
    Gyro *m_gyro;
};
