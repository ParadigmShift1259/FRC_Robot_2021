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

    /// Begins and stops parallel thread detection of input
    void StartDetection();
    void EndDetection();

    /// Returns whether the cycler is at the launch position
    bool AtPosition();
    /// Resets the triggered sensor back to false
    void ResetSensor();

private:    
    CANSparkMax m_feedermotor;
    TalonSRX m_turntablemotor;
    DigitalInput m_sensor;
    bool m_triggeredsensor;
};