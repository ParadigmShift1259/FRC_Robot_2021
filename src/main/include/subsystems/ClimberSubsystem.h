
#pragma once

#include <frc/SlewRateLimiter.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Spark.h>

#include "Constants.h"
#include "SwerveModule.h"

class ClimberSubsystem : public frc2::SubsystemBase
{
public:

    ClimberSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the climber at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Run(double speed);

private:    
    frc::Spark m_motor;
};
