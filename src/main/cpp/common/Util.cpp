
#include "common/Util.h"

// Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
double Util::ZeroTo2PiRads(double theta)
{
    theta = fmod(theta, 2 * wpi::math::pi);
    if (theta < 0)
        theta += 2 * wpi::math::pi;
        
    return theta;
}

// Convert any angle theta in degrees to its equivalent on the interval [0, 360]
double Util::ZeroTo360Degs(double theta)
{
    theta = fmod(theta, 360.0);
    if (theta < 0)
        theta += 360.0;
        
    return theta;
}

// Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
double Util::NegPiToPiRads(double theta)
{
    theta = ZeroTo2PiRads(theta);
    if (theta > wpi::math::pi)
        theta -= 2 * wpi::math::pi;
    else if (theta < -1.0 * wpi::math::pi)
        theta += 2 * wpi::math::pi;
    
    return theta;
}