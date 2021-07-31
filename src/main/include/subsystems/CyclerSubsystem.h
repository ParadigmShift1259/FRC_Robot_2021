#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include <frc/DigitalInput.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>

#pragma GCC diagnostic pop

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#include "Constants.h"
#include "common/Util.h"

using namespace std;
using namespace frc;
using namespace rev;

class CyclerSubsystem : public frc2::SubsystemBase
{
public:
    CyclerSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Turns the TurnTable at speed
    /// \param speed        Speed used in turning, between -1.0 and 1.0 with 0.0 as stopped
    void SetTurnTable(double speed);
    
    /// Turns the Feeder at speed
    /// \param speed        Speed used in turning, between -1.0 and 1.0 with 0.0 as stopped
    void SetFeeder(double speed);

    /// Begins parallel thread detection of input
    void StartDetection();\
    /// Begins parallel thread detection of input
    void EndDetection();

    /// Returns whether the cycler is at the launch position
    /// Sensor was removed on robot, now always returns true
    /// \return Whether or not the turntable sensor is at position
    bool AtPosition();
    /// Resets the triggered sensor back to false
    /// Function does nothing, as sensor was removed
    void ResetSensor();

private:
    /// NEO that feeds balls from turntable into shooter
    CANSparkMax m_feedermotor;
    /// 775 that shuffles balls around in the turntable
    TalonSRX m_turntablemotor;
    /// Sensor (REMOVED), used for detecting a specific position on the turntable
    DigitalInput m_sensor;
    /// Whether or not @ref m_sensor has been triggered, stays true until reset
    bool m_triggeredsensor;
    /// Unused boolean
    bool m_interruptsenabled;
};