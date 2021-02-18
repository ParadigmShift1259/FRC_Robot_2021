#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

#include "Constants.h"

class DriveDefault : public frc2::CommandHelper<frc2::CommandBase, DriveDefault> {
public:
    DriveDefault(DriveSubsystem* subsystem, std::function<double()> x, std::function<double()> y, std::function<double()> rot,
                std::function<double()> rotx, std::function<double()> roty, std::function<bool()> fieldRelative);

    void Execute() override;

 private:
    DriveSubsystem* m_drive;
    std::function<double()> m_x;
    std::function<double()> m_y;
    std::function<double()> m_rot;
    std::function<double()> m_rotx;
    std::function<double()> m_roty;
    std::function<bool()> m_fieldRelative;
};