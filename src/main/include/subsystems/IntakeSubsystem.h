
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Spark.h>

#include "Constants.h"

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
    frc::Spark m_motor;
};
