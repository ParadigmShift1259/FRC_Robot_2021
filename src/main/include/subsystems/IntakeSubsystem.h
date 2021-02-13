
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix.h>

#include "Constants.h"
#include "SwerveModule.h"

class IntakeSubsystem : public frc2::SubsystemBase
{
public:

    IntakeSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the intake at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Set(double speed);

private:    
    TalonSRX m_motor;
};
