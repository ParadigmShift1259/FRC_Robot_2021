#include "commands/HomeTarget.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

HomeTarget::HomeTarget(FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood,
                        VisionSubsystem* vision, bool* turretready, 
                        bool* firing, bool* finished)
 : m_flywheel(flywheel)
 , m_turret(turret)
 , m_hood(hood)
 , m_vision(vision)
 , m_turretready(turretready)
 , m_firing(firing)
 , m_finished(finished)
{
    AddRequirements({flywheel, turret, hood, vision});
    *m_turretready = false;
    *m_firing = false;
    *m_finished = false;
}

void HomeTarget::Initialize()
{
    *m_turretready = false;
    *m_firing = false;
    *m_finished = false;
}

void HomeTarget::Execute()
{
    // Homes flywheel, turret, and hood to the right angles through a formula
    // SmartDashboard::PutBoolean("TEST_VIS_ACTIVE", m_vision->GetActive());
    if (!m_vision->GetActive())
        return;

    double distance = m_vision->GetDistance();
    //y\ =\ 1687.747+15.8111x-0.0594079x^{2}+0.00008292342x^{3}
    // Increased flywheel at upper ends 3/18/21
    //y\ =\ 1687.747+15.8111x-0.058079x^{2}+0.00008892342x^{3}
    double flywheelspeed = 1687.747 + 15.8111 * distance - 0.058079 * pow(distance, 2) + 0.00008892342 * pow(distance, 3);
    flywheelspeed *= FlywheelConstants::kHomingRPMMultiplier;
    if (*m_firing)
        flywheelspeed *= FlywheelConstants::kFiringRPMMultiplier;
    // Quintic regression calculated 3/27
    // https://mycurvefit.com/
    //y=11.20831-0.2645223*x+0.002584349*x^{2}-0.00001250923*x^{3}+2.986403\cdot10^{-8}*x^{4}-2.81104\cdot10^{-11}*x^{5}
    double hoodangle = 11.20831 - 0.2645223 * distance + 0.002584349 * pow(distance, 2) - 0.00001250923 * pow(distance, 3) + 2.986403E-8 * pow(distance, 4) - 2.81104E-11 * pow(distance, 5);

    m_turret->TurnToRelative(m_vision->GetAngle());
    m_flywheel->SetRPM(flywheelspeed);
    m_hood->Set(hoodangle);

    SmartDashboard::PutBoolean("D_FIRE_AT_RPM", m_flywheel->IsAtRPM());
    SmartDashboard::PutBoolean("D_FIRE_AT_SET", m_turret->isAtSetpoint());

    // if at position, set turret ready to true
    if (m_flywheel->IsAtRPMPositive() && m_turret->isAtSetpoint()) {
        *m_turretready = true;
    }
}

bool HomeTarget::IsFinished()
{
    // SmartDashboard::PutBoolean("TEST_FIRE_FINISIHED", *m_finished);
    return *m_finished;
}

void HomeTarget::End(bool interrupted) {
    *m_finished = false;
    *m_turretready = false;
    m_flywheel->SetRPM(FlywheelConstants::kIdleRPM);
    m_hood->Set(HoodConstants::kMax);
    m_turret->TurnTo(TurretConstants::kStartingPositionDegrees);
}