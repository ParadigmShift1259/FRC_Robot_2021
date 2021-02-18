
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Servo.h>

#include "Constants.h"
#include "SwerveModule.h"

class HoodSubsystem : public frc2::SubsystemBase
{
public:

    HoodSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Set hood to certain position
    /// \param position         Servo rotation, ranging from [0, 1]
    void Set(double position);

private:    
    frc::Servo m_servo;
};
