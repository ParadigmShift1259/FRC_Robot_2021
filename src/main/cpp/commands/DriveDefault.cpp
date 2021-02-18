
#include "commands/DriveDefault.h"
#include "Constants.h"

using namespace DriveConstants;
DriveDefault::DriveDefault( DriveSubsystem* subsystem, 
                            std::function<double()> x, std::function<double()> y, std::function<double()> rot,
                            std::function<double()> rotx, std::function<double()> roty, std::function<bool()> fieldRelative)
 : m_drive(subsystem)
 , m_x(x)
 , m_y(y)
 , m_rot(rot)
 , m_rotx(rotx)
 , m_roty(roty)
 , m_fieldRelative(fieldRelative)
{
    AddRequirements({subsystem});
}

void DriveDefault::Execute() {
    // up is xbox joystick y pos
    // left is xbox joystick x pos

    auto xInput = m_x();
    auto yInput = m_y();
    auto rotInput = m_rot();
    auto xRot = m_rotx();
    auto yRot = m_roty();
    bool fieldRelative = m_fieldRelative();
    
    if (sqrt(pow(xRot, 2) + pow(yRot, 2)) < OIConstants::kDeadzoneAbsRot) {
        xRot = 0;
        yRot = 0;
    }

    if (fieldRelative)
    {
        m_drive->RotationDrive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                    units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                    xRot,
                    yRot,
                    fieldRelative);
    }
    else 
    {
        m_drive->Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                    units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                    units::radians_per_second_t(rotInput),
                    fieldRelative);
    }
}
