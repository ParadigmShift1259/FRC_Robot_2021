/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardWidget.h>
#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

#include "Constants.h"

// Removes deprecated warning for CANEncoder and CANPIDController
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#ifdef Mk2

SwerveModule::SwerveModule(int driveMotorChannel, 
                           int turningMotorChannel,
                           const int turningEncoderPort,
                           bool driveMotorReversed,
                           double offset,
                           const std::string& name,
                           Logger& log)
    : m_offset(offset)
    , m_name(name)
    , m_driveMotor(driveMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_turningMotor(turningMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_driveEncoder(m_driveMotor)
    , m_turnNeoEncoder(m_turningMotor)
    , m_turningEncoder(turningEncoderPort)
    , m_logData(c_headerNamesSwerveModule, false, name)
    , m_log(log)
{
    m_driveMotor.SetSmartCurrentLimit(ModuleConstants::kMotorCurrentLimit);
    m_turningMotor.SetSmartCurrentLimit(ModuleConstants::kMotorCurrentLimit);

    // Set up GetVelocity() to return meters per sec instead of RPM
    m_driveEncoder.SetVelocityConversionFactor(wpi::math::pi * ModuleConstants::kWheelDiameterMeters / (ModuleConstants::kDriveGearRatio * 60.0));
    m_turnNeoEncoder.SetPositionConversionFactor(2 * wpi::math::pi / ModuleConstants::kTurnMotorRevsPerWheelRev);
    
    m_driveMotor.SetInverted(driveMotorReversed);
    m_turningMotor.SetInverted(false);

    m_drivePidParams.Load(m_drivePIDController);
    m_turnPidParams.Load(m_turnPIDController);

    double initPosition = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offset);
    m_turnNeoEncoder.SetPosition(initPosition); // Tell the encoder where the absolute encoder is

    ShuffleboardTab& tab = Shuffleboard::GetTab("AbsEncTuning");
    std::string nteName = m_name + " offset";
    wpi::StringMap<std::shared_ptr<nt::Value>> sliderPropMap
    {
          std::make_pair("Min", nt::Value::MakeDouble(0.0))
        , std::make_pair("Max", nt::Value::MakeDouble(2 * wpi::math::pi))
        , std::make_pair("Block increment", nt::Value::MakeDouble(wpi::math::pi / 180))
    };
    m_nteAbsEncTuningOffset    = tab.Add(nteName, m_offset)
                                    .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                                    .WithProperties(sliderPropMap)
                                    .GetEntry();
    nteName = m_name + " voltage";
    m_nteAbsEncTuningVoltage    = tab.Add(nteName, m_turningMotor.GetAnalog().GetVoltage())
                                    .WithWidget(frc::BuiltInWidgets::kVoltageView)
                                    .GetEntry();
}

#pragma GCC diagnostic pop

frc::SwerveModuleState SwerveModule::GetState()
{
    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offset);
    return {meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(radian_t(angle))};
}

void SwerveModule::Periodic()
{
    double absAngle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offset);
    SmartDashboard::PutNumber("D_SM " + m_name, absAngle);
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    #ifdef TUNE_MODULE
    // Retrieving turn PID values from SmartDashboard
    m_drivePidParams.LoadFromNetworkTable(m_drivePIDController);
    m_turnPidParams.LoadFromNetworkTable(m_turnPIDController);
    #endif

    // Find absolute encoder and NEO encoder positions
    double absAngle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offset);
    double currentPosition = m_turnNeoEncoder.GetPosition();

    // Calculate new turn position given current Neo position, current absolute encoder position, and desired state position
    bool bOutputReverse = false;
    double minTurnRads = MinTurnRads(absAngle, state.angle.Radians().to<double>(), bOutputReverse);
    double direction = 1.0;
    // Inverts the drive motor if going in the "backwards" direction on the swerve module
    if (bOutputReverse)
        direction = -1.0;

    // Set position reference of turnPIDController
    double newPosition = currentPosition + minTurnRads;

    #ifdef DISABLE_DRIVE
    m_drivePIDController.SetReference(0.0, rev::ControlType::kVelocity);
    #else
    m_drivePIDController.SetReference(state.speed.to<double>(), rev::ControlType::kVelocity);
    #endif

    // Set the angle unless module is coming to a full stop
    if (state.speed.to<double>() != 0.0)
    {
        m_turnPIDController.SetReference(newPosition, rev::ControlType::kPosition);
    }

    const std::string FuncModule = "Swerve" + m_name;
    m_logData[ESwerveModuleLogData::eDesiredAngle] = state.angle.Radians().to<double>();
    m_logData[ESwerveModuleLogData::eTurnEncVolts] = m_turningEncoder.GetVoltage();
    m_logData[ESwerveModuleLogData::eTurnEncAngle] = absAngle;
    m_logData[ESwerveModuleLogData::eMinTurnRads] = minTurnRads;
    m_logData[ESwerveModuleLogData::eTurnNeoPidRefPos] = newPosition;
    m_logData[ESwerveModuleLogData::eTurnNeoEncoderPos] = currentPosition;
    m_logData[ESwerveModuleLogData::eTurnOutputDutyCyc] = m_turningMotor.GetAppliedOutput();
    m_logData[ESwerveModuleLogData::eDrivePidRefSpeed] = state.speed.to<double>();
    m_logData[ESwerveModuleLogData::eDriveEncVelocity] = m_driveEncoder.GetVelocity();
    m_logData[ESwerveModuleLogData::eDriveOutputDutyCyc] = m_driveMotor.GetAppliedOutput();
    m_log.logData<ESwerveModuleLogData>(FuncModule.c_str(), m_logData);
}

void SwerveModule::ResetEncoders()
{
    m_driveEncoder.SetPosition(0.0); 
}

double SwerveModule::VoltageToRadians(double voltage, double offset)
{
    double angle = fmod(voltage * DriveConstants::kTurnVoltageToRadians - m_offset + 2 * wpi::math::pi, 2 * wpi::math::pi);
    angle = 2 * wpi::math::pi - angle;

    return angle;
}

double SwerveModule::VoltageToDegrees(double voltage, double offSet)
{
    double angle = fmod(voltage * DriveConstants::KTurnVoltageToDegrees - offSet + 360.0, 360.0);

    return angle;
}

// Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
double SwerveModule::ZeroTo2PiRads(double theta)
{
    theta = fmod(theta, 2 * wpi::math::pi);
    if (theta < 0)
        theta += 2 * wpi::math::pi;
        
    return theta;
}

// Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
double SwerveModule::NegPiToPiRads(double theta)
{
    theta = ZeroTo2PiRads(theta);
    if (theta > wpi::math::pi)
        theta -= 2 * wpi::math::pi;
    else if (theta < -1.0 * wpi::math::pi)
        theta += 2 * wpi::math::pi;
    
    return theta;
}

// Determine the smallest magnitude delta angle that can be added to initial angle that will 
// result in an angle equivalent (but not necessarily equal) to final angle. 
// All angles in radians
// 
// init final   angle1   angle2 
double SwerveModule::MinTurnRads(double init, double final, bool& bOutputReverse)
{
    init = ZeroTo2PiRads(init);
    final = ZeroTo2PiRads(final);

    // The shortest turn angle may be acheived by reversing the motor output direction
    double angle1 = final - init;
    double angle2 = final + wpi::math::pi - init;

    angle1 = NegPiToPiRads(angle1);
    angle2 = NegPiToPiRads(angle2);

    // Choose the smallest angle and determine reverse flag
    //TODO: FINISHED ROBOT TUNING
    // Eventually prefer angle 1 always during high speed to prevent 180s
    
    SmartDashboard::PutNumber("D_T_SM_Speed", GetState().speed.to<double>());

    bOutputReverse = false;

    return angle1;
    // Ignoring bottom code because not working for now, will implement later

    if (GetState().speed.to<double>() > DriveConstants::kMinTurnPrioritySpeed)
    {
        bOutputReverse = false;

        return angle1;
    }
    else
    if (GetState().speed.to<double>() < -1.0 * DriveConstants::kMinTurnPrioritySpeed)
    {
        bOutputReverse = true;

        return angle2;
    }

    if (fabs(angle1) <= 2 * fabs(angle2))
    {
        bOutputReverse = false;

        return angle1;
    } 
    else
    {
        bOutputReverse = true;

        return angle2;
    }
}

#endif