
#pragma once

#include <wpi\math>

#include "Constants.h"

using namespace std;

class Util
{
public:
    // Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
    static double ZeroTo2PiRads(double theta);

    // Convert any angle theta in degrees to its equivalent on the interval [0, 360]
    static double ZeroTo360Degs(double theta);

    // Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
    static double NegPiToPiRads(double theta);
};
