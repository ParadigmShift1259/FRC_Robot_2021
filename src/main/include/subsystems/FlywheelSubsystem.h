
#pragma once

#include <frc2/command/SubsystemBase.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>

#pragma GCC diagnostic pop

#include "units/length.h"
#include "units/time.h"
#include "units/voltage.h"
#include <frc/controller/SimpleMotorFeedforward.h>

#include "Constants.h"

using namespace rev;
using namespace std;
using namespace frc;

class FlywheelSubsystem : public frc2::SubsystemBase
{
public:

    FlywheelSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Sets the flywheel to a desired rpm
    void SetRPM(double rpm);

    /// Returns whether or not the flywheel is at the desired RPM
    bool isAtRPM();

private:
    CANSparkMax m_flywheelmotor;
    CANPIDController m_flywheelPID;
    CANEncoder m_flywheelencoder;

    SimpleMotorFeedforward<units::meters> m_flywheelFF;

    // Current desired setpoint of the flywheel in RPM
    double m_setpoint;
};
