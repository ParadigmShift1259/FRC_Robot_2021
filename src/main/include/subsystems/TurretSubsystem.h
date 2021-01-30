
#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix.h>

#include <units\units.h>

#include "Constants.h"

using namespace std;
using namespace frc;

class TurretSubsystem : public frc2::SubsystemBase
{
public:

    TurretSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

protected:
    /// Converts motor ticks into turret rotation, in degrees
    /// \param ticks        Number of ticks to be converted
    double TicksToDegrees(double ticks);

    /// Converts turret rotation in degrees into motor ticks
    /// \param degrees      Number of degrees to be converted
    double DegreesToTicks(double degrees);

private:    
    WPI_TalonSRX m_turretmotor;
};
