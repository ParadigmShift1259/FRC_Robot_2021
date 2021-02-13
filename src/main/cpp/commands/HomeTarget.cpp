#include "commands/HomeTarget.h"
#include "Constants.h"

HomeTarget::HomeTarget(FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood,
                        VisionSubsystem* vision, bool* turretready)
 : m_flywheel(flywheel)
 , m_turret(turret)
 , m_hood(hood)
 , m_vision(vision)
 , m_turretready(turretready)
{
  AddRequirements({flywheel, turret, hood, vision});
}

void HomeTarget::Execute()
{
    // Homes flywheel, turret, and hood to the right angles through a formula
    if (!m_vision->GetActive())
        return;

    double distance = m_vision->GetDistance();
    //y\ =\ 1687.747+15.8111x-0.0594079x^{2}+0.00008292342x^{3}
    double flywheelspeed = 1687.747 + 15.8111 * distance - 0.0594079 * pow(distance, 2) + 0.00008292342 * pow(distance, 3);
    double hoodangle = 0.06286766 + (175598.7 - 0.06286766) / (1 + pow((distance / 0.6970016), 2.811798));

    m_turret->TurnToRelative(m_vision->GetAngle());
    m_flywheel->SetRPM(flywheelspeed);
    m_hood->Set(hoodangle);

    // if at position, set turret ready to true
}