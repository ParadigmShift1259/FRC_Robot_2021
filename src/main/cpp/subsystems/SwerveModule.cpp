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
#include <iostream>

#include "Constants.h"

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

frc::SwerveModuleState SwerveModule::GetState()
{
//    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offset);
    double angle = m_turnNeoEncoder.GetPosition();
    return {meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(radian_t(angle))};
}

void SwerveModule::Periodic()
{
    double absAngle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offset);
    SmartDashboard::PutNumber(m_name, absAngle);

    /* Used for tuning SwerveModule Turn PID
    double P = SmartDashboard::GetNumber("TEST_TurnP", 0);
    double I = SmartDashboard::GetNumber("TEST_TurnI", 0);
    double D = SmartDashboard::GetNumber("TEST_TurnD", 0);

    m_turnPIDController.SetP(P);
    m_turnPIDController.SetI(I);
    m_turnPIDController.SetD(D);
    */
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    // Retrieving turn PID values from SmartDashboard
    m_drivePidParams.LoadFromNetworkTable(m_drivePIDController);
    m_turnPidParams.LoadFromNetworkTable(m_turnPIDController);

    // Find absolute encoder and NEO encoder positions
    double absAngle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offset);
    double currentPosition = m_turnNeoEncoder.GetPosition();

    // Calculate new turn position given current Neo position, current absolute encoder position, and desired state position
    bool bOutputReverse = false;
    double minTurnRads = MinTurnRads(currentPosition, state.angle.Radians().to<double>(), bOutputReverse);
    double direction = 1.0;
    // Inverts the drive motor if going in the "backwards" direction on the swerve module
    if (bOutputReverse)
        direction = -1.0;

    // Set position reference of turnPIDController
    double newPosition = currentPosition + minTurnRads;

//#define TUNE_ABS_ENC
#ifdef TUNE_ABS_ENC
    m_drivePIDController.SetReference(0.0, rev::ControlType::kVelocity);
#endif

    // If we're stopping then stop the drive motors and leave the angle alone
    if (state.speed.to<double>() == 0.0)
    {
#ifndef TUNE_ABS_ENC
        m_drivePIDController.SetReference(state.speed.to<double>(), rev::ControlType::kVelocity);
#endif
    }
    else
    {
        // Otherwise set the angle
        m_turnPIDController.SetReference(newPosition, rev::ControlType::kPosition);
    }
    
    // Let the turn complete before we activate the motor
    // 1/22/21
    if (fabs(currentPosition - newPosition) < DriveConstants::kTurnValidationDistance)
    {
        // Set velocity reference of drivePIDController
#ifndef TUNE_ABS_ENC
        m_drivePIDController.SetReference(direction * state.speed.to<double>(), rev::ControlType::kVelocity);
#endif
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
#ifdef TUNE_ABS_ENC
    offset = m_nteAbsEncTuningOffset.GetDouble(m_offset);
#endif
    m_nteAbsEncTuningVoltage.SetDouble(voltage);
    double angle = fmod(voltage * DriveConstants::kTurnVoltageToRadians - offset + 2 * wpi::math::pi, 2 * wpi::math::pi);
    // Negating angle here becuse analog encoder is increasing clockwise. TO DO: just make kTurnVoltageToRadians negative!
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
    /*
    init = ZeroTo2PiRads(init);
    final = ZeroTo2PiRads(final);

    double turn = final - init;
    if (turn > M_PI)
        turn -= 2 * M_PI;
    else if (turn < -M_PI)
        turn += 2 * M_PI;
        
    bOutputReverse = false;
    return turn;
    */
    
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
    
    SmartDashboard::PutNumber("kTestSpeed", GetState().speed.to<double>());

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
