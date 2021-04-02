/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule2.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardWidget.h>
#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

#include "Constants.h"

SwerveModule2::SwerveModule2(int driveMotorChannel, 
                           int turningMotorChannel,
                           GetPulseWidthCallback pulseWidthCallback,
                           CANifier::PWMChannel pwmChannel,
                           bool driveMotorReversed,
                           double offset,
                           const std::string& name)
    : m_offset(offset)
    , m_name(name)
    , m_driveMotor(driveMotorChannel)
    , m_turningMotor(turningMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_pulseWidthCallback(pulseWidthCallback)
    , m_pwmChannel(pwmChannel)
{
    StatorCurrentLimitConfiguration statorLimit { true, ModuleConstants::kMotorCurrentLimit, ModuleConstants::kMotorCurrentLimit, 2 };
    m_driveMotor.ConfigStatorCurrentLimit(statorLimit);
    SupplyCurrentLimitConfiguration supplyLimit { true, ModuleConstants::kMotorCurrentLimit, ModuleConstants::kMotorCurrentLimit, 2 };
    m_driveMotor.ConfigSupplyCurrentLimit(supplyLimit);
    m_turningMotor.SetSmartCurrentLimit(ModuleConstants::kMotorCurrentLimit);

    // Set up GetVelocity() to return meters per sec instead of RPM
    m_turnRelativeEncoder.SetPositionConversionFactor(Math::kTau / ModuleConstants::kTurnMotorRevsPerWheelRev);
    
    m_driveMotor.SetInverted(driveMotorReversed ? TalonFXInvertType::CounterClockwise : TalonFXInvertType::Clockwise);
    m_turningMotor.SetInverted(false);
    m_driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    m_turnPIDController.SetFeedbackDevice(m_turnRelativeEncoder);
    
    m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_driveMotor.SetNeutralMode(NeutralMode::Brake);

    EncoderToRadians();
    m_turnRelativeEncoder.SetPosition(m_absAngle); // Tell the encoder where the absolute encoder is

    m_drivePidParams.Load(m_driveMotor);
    m_turnPidParams.Load(m_turnPIDController);

    m_timer.Reset();
    m_timer.Start();
}

frc::SwerveModuleState SwerveModule2::GetState()
{
    EncoderToRadians();
    return { CalcMetersPerSec(), frc::Rotation2d(radian_t(m_absAngle))};
}

void SwerveModule2::Periodic()
{
    EncoderToRadians();

    if (m_timer.Get() < 2)
    {
        // printf( "Seeding the relative encoder with absolute encoder: %.3f %.3f %.3f \n", 
        //         fabs(m_absAngle - m_turnRelativeEncoder.GetPosition()), 
        //         m_absAngle, 
        //         m_turnRelativeEncoder.GetPosition());
        m_turnRelativeEncoder.SetPosition(m_absAngle); // Tell the relative encoder where the absolute encoder is
    }

    SmartDashboard::PutNumber("D_SM_Rel " + m_name, m_turnRelativeEncoder.GetPosition());
    SmartDashboard::PutNumber("D_SM_Abs " + m_name, m_absAngle);
    SmartDashboard::PutNumber("D_SM_AbsDiff " + m_name, m_turnRelativeEncoder.GetPosition() - m_absAngle);
    SmartDashboard::PutNumber("D_SM_MPS " + m_name, CalcMetersPerSec().to<double>());
    SmartDashboard::PutNumber("D_SM_TP100MS " + m_name, m_driveMotor.GetSelectedSensorVelocity());
    SmartDashboard::PutNumber("D_SM_RelToAbsError " + m_name, m_absAngle - m_turnRelativeEncoder.GetPosition());
}

void SwerveModule2::SetDesiredState(frc::SwerveModuleState &state)
{
    // Retrieving turn PID values from SmartDashboard
    m_drivePidParams.LoadFromNetworkTable(m_driveMotor);
    m_turnPidParams.LoadFromNetworkTable(m_turnPIDController);

    // Find absolute encoder and NEO encoder positions
    //EncoderToRadians();
    double currentPosition = m_turnRelativeEncoder.GetPosition();

    // Calculate new turn position given current Neo position, current absolute encoder position, and desired state position
    bool bOutputReverse = false;
    double minTurnRads = MinTurnRads(currentPosition, state.angle.Radians().to<double>(), bOutputReverse);
    double direction = 1.0; // Sent to TalonFX
    if (bOutputReverse)
        direction = -1.0;

    // Set position reference of turnPIDController
    double newPosition = currentPosition + minTurnRads;

    #ifdef DISABLE_DRIVE
    m_driveMotor.Set(TalonFXControlMode::Velocity, 0.0);
    #else
    m_driveMotor.Set(TalonFXControlMode::Velocity, direction * CalcTicksPer100Ms(state.speed));
    #endif

    // Set the angle unless module is coming to a full stop
    if (state.speed.to<double>() != 0.0)
        m_turnPIDController.SetReference(newPosition, rev::ControlType::kPosition);


    SmartDashboard::PutNumber("D_SM_SetpointMPS " + m_name, state.speed.to<double>());
    SmartDashboard::PutNumber("D_SM_Error " + m_name, newPosition - m_turnRelativeEncoder.GetPosition());
}

void SwerveModule2::ResetEncoders()
{
    m_driveMotor.SetSelectedSensorPosition(0.0);
}

void SwerveModule2::EncoderToRadians()
{
    double pulseWidth = m_pulseWidthCallback(m_pwmChannel);
    m_absAngle = fmod((pulseWidth - m_offset) * DriveConstants::kPulseWidthToRadians + Math::kTau, Math::kTau);
    SmartDashboard::PutNumber("D_SM_PW " + m_name, pulseWidth);
    // Convert CW to CCW? m_absAngle = Math::kTau - m_absAngle;
}

void SwerveModule2::ResetRelativeToAbsolute()
{
    // printf( "Seeding the relative encoder with absolute encoder: %.3f %.3f %.3f \n", 
    //             fabs(m_absAngle - m_turnRelativeEncoder.GetPosition()), 
    //             m_absAngle, 
    //             m_turnRelativeEncoder.GetPosition());
    m_turnRelativeEncoder.SetPosition(m_absAngle);
}

// Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
double SwerveModule2::ZeroTo2PiRads(double theta)
{
    theta = fmod(theta, Math::kTau);
    if (theta < 0)
        theta += Math::kTau;
        
    return theta;
}

// Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
double SwerveModule2::NegPiToPiRads(double theta)
{
    theta = ZeroTo2PiRads(theta);
    if (theta > wpi::math::pi)
        theta -= Math::kTau;
    else if (theta < -1.0 * wpi::math::pi)
        theta += Math::kTau;
    
    return theta;
}

// Determine the smallest magnitude delta angle that can be added to initial angle that will 
// result in an angle equivalent (but not necessarily equal) to final angle. 
// All angles in radians
// 
// init final   angle1   angle2 
double SwerveModule2::MinTurnRads(double init, double final, bool& bOutputReverse)
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
    // if (fabs(angle1) <= 2 * fabs(angle2))
    // {
        bOutputReverse = false;

        return angle1;
    // } 
    // else
    // {
    //     bOutputReverse = true;

    //     return angle2;
    // }
}

meters_per_second_t SwerveModule2::CalcMetersPerSec()
{
   double ticksPer100ms = m_driveMotor.GetSelectedSensorVelocity();
   return meters_per_second_t(ModuleConstants::kDriveEncoderMetersPerSec * ticksPer100ms);
}

double SwerveModule2::CalcTicksPer100Ms(meters_per_second_t speed)
{
   return speed.to<double>() / ModuleConstants::kDriveEncoderMetersPerSec;
}