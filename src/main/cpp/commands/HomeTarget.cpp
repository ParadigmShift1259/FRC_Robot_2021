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
}

void HomeTarget::Initialize()
{
    *m_finished = false;
    *m_turretready = false;
}

void HomeTarget::Execute()
{
    // Homes flywheel, turret, and hood to the right angles through a formula
    printf("Running Home Target ...\n");
    SmartDashboard::PutBoolean("TEST_VIS_ACTIVE", m_vision->GetActive());
    if (!m_vision->GetActive())
        return;

    double distance = m_vision->GetDistance();
    //y\ =\ 1687.747+15.8111x-0.0594079x^{2}+0.00008292342x^{3}
    // Increased flywheel at upper ends 3/18/21
    //y\ =\ 1687.747+15.8111x-0.058079x^{2}+0.00008892342x^{3}
    double flywheelspeed = 1687.747 + 15.8111 * distance - 0.058079 * pow(distance, 2) + 0.00008892342 * pow(distance, 3);
    if (*m_firing)
        flywheelspeed *= FlywheelConstants::kFiringRPMMultiplier;
    double hoodangle = 0.06286766 + (175598.7 - 0.06286766) / (1 + pow((distance / 0.6970016), 2.811798));

    m_turret->TurnToRelative(m_vision->GetAngle());
    m_flywheel->SetRPM(flywheelspeed);
    //m_hood->Set(hoodangle);

    SmartDashboard::PutBoolean("TEST_AT_RPM", m_flywheel->IsAtRPM());
    SmartDashboard::PutBoolean("TEST_AT_SET", m_turret->isAtSetpoint());

    // if at position, set turret ready to true
    if (m_flywheel->IsAtRPMPositive() && m_turret->isAtSetpoint()) {
        *m_turretready = true;
    }
}

bool HomeTarget::IsFinished()
{
    SmartDashboard::PutBoolean("TEST_FIRE_FINISIHED", *m_finished);
    return *m_finished;
}

void HomeTarget::End(bool interrupted) {
    *m_finished = false;
    *m_turretready = false;
    m_flywheel->SetRPM(FlywheelConstants::kIdleRPM);
    m_hood->Set(0);
    m_turret->TurnTo(TurretConstants::kStartingPositionDegrees);
}