
#pragma once

#include <frc2/command/SubsystemBase.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <wpi/math>
#include "Constants.h"

using namespace std;
using namespace frc;

class VisionSubsystem : public frc2::SubsystemBase
{
public:
    enum BallDirection : int
    {
        kLeft,
        kRight,
        kDefault
    };
    
    VisionSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Determine whether or not the Vision Subsystem is giving accurate values
    bool GetActive();
    /// Retrieves the distance calculation from the target via the limelight
    double GetDistance();
    /// Retrieves left, right, or default for direction of ball to be used for enumeration
    BallDirection GetDirection();
    /// Retrieves the angle calculation from the target via the limelight
    double GetAngle();
    /// Turns the limelight LED on or off
    /// \param on        Boolean where true = LED on
    void SetLED(bool on);

protected:
    /// Converts degrees to radians
    /// \param degrees Degrees to convert
    double DegreesToRadians(double degrees);

private:    
    shared_ptr<NetworkTable> m_dashboard;
    int m_camerachoice;
    
    shared_ptr<NetworkTable> m_networktable;
    bool m_led;
    double m_tx;
    double m_ty;
    double m_ta;
    double m_ts;
    vector<double> m_tcornx;
    vector<double> m_tcorny;
    bool m_active;

    double m_verticalangle;

    double m_distance;
    double m_horizontalangle;
    int m_direction;

    vector<double> m_averagedistance;
    double m_avgdistance;
    vector<double> m_averageangle;
    

};
