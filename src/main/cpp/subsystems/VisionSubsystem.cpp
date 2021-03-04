#include "subsystems/VisionSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace VisionConstants;

VisionSubsystem::VisionSubsystem() 
 : m_networktable(nt::NetworkTableInstance::GetDefault().GetTable("limelight"))
 , m_dashboard (nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard"))
 , m_camerachoice(0)
 , m_led(true)
{
    m_tx = 0;
    m_ty = 0;
    m_ta = 0;
    m_ts = 0;
    m_active = false;

    m_verticalangle = 0;

    m_distance = 0;
    m_horizontalangle = 0;

    m_averagedistance.reserve(3);
    m_avgdistance = 0;

    m_averageangle[0] = 0;
    m_averageangle[1] = 0;
    m_averageangle[2] = 0;

    SetLED(true);
}

void VisionSubsystem::Periodic()
{
    m_dashboard->PutNumber("cameraFeed", m_camerachoice);

    m_active = m_networktable->GetNumber("tv", 0);

    if (!m_led)
        m_active = false;

    if (!m_active)
    {
        m_averagedistance.clear();
        m_averageangle[0] = 0;
        m_averageangle[1] = 0;
        m_averageangle[2] = 0;
        return;
    }

    m_tx = m_networktable->GetNumber("tx", 0.0);
    m_ty = m_networktable->GetNumber("ty", 0.0);

    /*
    m_ta = m_networktable->GetNumber("ta", 0.0);
    m_ts = m_networktable->GetNumber("ts", 0.0);
    m_tcornx = m_networktable->GetNumberArray("tcornx", 0.0);
    m_tcorny = m_networktable->GetNumberArray("tcorny", 0.0);
    */

    m_verticalangle = kMountingAngle + m_ty;    // in degrees
    
    m_distance = (kTargetHeight - kMountingHeight) / tanf(DegreesToRadians(m_verticalangle));
    m_horizontalangle = m_tx;

    m_averagedistance.push_back(m_distance);

    if (m_averagedistance.size() > 3)
        m_averagedistance.erase(m_averagedistance.begin());

    for (int i = 0; i < 2; i++)
    {
        m_averageangle[i] = m_averageangle[i + 1];
    }
    if ((m_distance < 110) || (m_distance > 380))
        m_active = false;

    m_averageangle[2] = m_horizontalangle;

    SmartDashboard::PutNumber("VIS0_Active", m_active);
    SmartDashboard::PutNumber("VIS1_Distance", m_distance);
    SmartDashboard::PutNumber("VIS2_Angle", m_horizontalangle);
    SmartDashboard::PutNumber("VIS3_Average Distance", m_avgdistance);
    SmartDashboard::PutNumber("VIS4_Average Angle", 
    (m_averageangle[0] + m_averageangle[1] + m_averageangle[2]) / 3);
}

bool VisionSubsystem::GetActive()
{
    return m_active;
}


double VisionSubsystem::GetDistance()
{
    double sum = 0.0;

    for(auto d:m_averagedistance)
        sum += d;
    
    m_avgdistance = sum / m_averagedistance.size();

    return m_avgdistance;
}


double VisionSubsystem::GetAngle()
{
    return m_horizontalangle;
}


void VisionSubsystem::SetLED(bool on)
{
    m_led = on;
    if (m_led)
    {
        // 3 forces limelight led on
        m_networktable->PutNumber("ledMode", 3);
    }
    else
    {
        // 1 forces limelight led off
        m_networktable->PutNumber("ledMode", 1);
    }
}

double VisionSubsystem::DegreesToRadians(double degrees)
{
    return degrees * 2 * wpi::math::pi / 360.0;
}