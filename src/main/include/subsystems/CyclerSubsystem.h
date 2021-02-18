// Cycler is the subsystem containing both the turntable and the feeder motor
// Consider both to be Talon SRXes
// Create a feeder motor and a turntable motor
// The turntable motor will have an encoder that will eventually be used to track a position
    // See Turret.cpp on how to configure that
// During initialization, make sure to set these settings:
/*
- SetNeutralMode (set to brake)
- SensorPhase (use constant)
- SetInverted (use constant)

For now, there is no need to include any PID settings (config PID functions or min/max outputs are not needed)
*/
// Create a SetFeeder and SetTurnTable function so both motors can be directly set
// Also include GetPosition for turntable


// After - Create two commands. Each will run the feeder motor for a designated amount of time at a button press
    // This below example shows how to run a a command WithTimeout, meaning that the command will last another X amount of seconds
    // after the press.
    /*
        frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA).WhenPressed(
            frc2::RunCommand(    
            [this, c_buttonInputSpeed] {
                m_drive.Drive(units::meters_per_second_t(-c_buttonInputSpeed),
                        units::meters_per_second_t(0),
                        units::radians_per_second_t(0),
                        false);
            },
            {&m_drive}
        ).WithTimeout(c_buttonInputTime));
    */
   // Since you'll be creating a command class, you can bind it simply as so:
   /*
   frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperLeft).WhenPressed(&m_yourCommand).WithTimeout(yourSeconds);
   */

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix.h>

#include <units\units.h>

#include "Constants.h"
#include "common/Util.h"

using namespace std;
using namespace frc;

class CyclerSubsystem : public frc2::SubsystemBase
{
public:
    CyclerSubsystem();

    /// Turns the TurnTable at speed
    /// \param speed        Speed used in turning, between -1.0 and 1.0 with 0.0 as stopped
    void SetTurnTable(double speed);
    
    /// Turns the Feeder at speed
    /// \param speed        Speed used in turning, between -1.0 and 1.0 with 0.0 as stopped
    void SetFeeder(double speed);

    /// Gets the TurnTable position
    double GetPosition();

protected:
    /// Converts motor ticks into turntable rotation, in degrees
    /// \param ticks        Number of ticks to be converted
    double TicksToDegrees(double ticks);

    /// Converts turntable rotation in degrees into motor ticks
    /// \param degrees      Number of degrees to be converted
    double DegreesToTicks(double degrees);


private:    
    WPI_TalonSRX m_feedermotor;
    WPI_TalonSRX m_turntablemotor;
};