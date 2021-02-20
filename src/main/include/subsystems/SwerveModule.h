/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <wpi/math>

#include <rev\CANSparkMax.h>
#include <rev\CANEncoder.h>

#include <ctre\Phoenix.h>

#include <string>

#include "Constants.h"
#include "Logger.h"

using namespace rev;
using namespace units;
using namespace ctre::phoenix::motorcontrol;

using GetPulseWidthCallback = std::function<double (CANifier::PWMChannel)>;

class TurnPidParams
{
    double m_p = DriveConstants::kTurnP;
    double m_i = DriveConstants::kTurnI;
    double m_d = DriveConstants::kTurnD;
    double m_iz = 0.0;
    double m_ff = 0.0;
    double m_max = 1.0;
    double m_min = -1.0;

public:
    void Load(CANPIDController& turnPIDController)
    {
        turnPIDController.SetP(m_p);
        turnPIDController.SetI(m_i);
        turnPIDController.SetD(m_d);
        turnPIDController.SetIZone(m_iz);
        turnPIDController.SetFF(m_ff);
        turnPIDController.SetOutputRange(m_min, m_max);
        frc::SmartDashboard::PutNumber("Turn P Gain", m_p);
        frc::SmartDashboard::PutNumber("Turn I Gain", m_i);
        frc::SmartDashboard::PutNumber("Turn D Gain", m_d);
        frc::SmartDashboard::PutNumber("Turn I Zone", m_iz);
        frc::SmartDashboard::PutNumber("Turn Feed Forward", m_ff);
        frc::SmartDashboard::PutNumber("Turn Max Output", m_max);
        frc::SmartDashboard::PutNumber("Turn Min Output", m_min);
    }

    void LoadFromNetworkTable(CANPIDController& turnPIDController)
    {
        double p = frc::SmartDashboard::GetNumber("Turn P Gain", 0.0);
        double i = frc::SmartDashboard::GetNumber("Turn I Gain", 0.0);
        double d = frc::SmartDashboard::GetNumber("Turn D Gain", 0.0);
        double iz = frc::SmartDashboard::GetNumber("Turn I Zone", 0.0);
        double ff = frc::SmartDashboard::GetNumber("Turn Feed Forward", 0.0);
        double max = frc::SmartDashboard::GetNumber("Turn Max Output", 0.0);
        double min = frc::SmartDashboard::GetNumber("Turn Min Output", 0.0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != m_p)) { turnPIDController.SetP(p); m_p = p; }
        if ((i != m_i)) { turnPIDController.SetI(i); m_i = i; }
        if ((d != m_d)) { turnPIDController.SetD(d); m_d = d; }
        if ((iz != m_iz)) { turnPIDController.SetIZone(iz); m_iz = iz; }
        if ((ff != m_ff)) { turnPIDController.SetFF(ff); m_ff = ff; }
        
        if ((max != m_max) || (min != m_min))
        { 
            turnPIDController.SetOutputRange(min, max);
            m_min = min;
            m_max = max; 
        }
    }
};

class DrivePidParams
{
    double m_p = 0.0;
    //double m_i = 0.0;
    double m_d = 0.0;
    double m_ff = 0.047619;
    double m_max = 1.0;
    double m_min = -1.0;

public:
   void Load(TalonFX& driveMotor)
    {
        driveMotor.Config_kP(0, m_p);
        // driveMotor.Config_kI(0, m_i);
        driveMotor.Config_kD(0, m_d);
        driveMotor.Config_kF(0, m_ff);
        driveMotor.ConfigPeakOutputForward(m_max);
        driveMotor.ConfigPeakOutputReverse(m_min);
        frc::SmartDashboard::PutNumber("Drive P Gain", m_p);
        //frc::SmartDashboard::PutNumber("Drive I Gain", m_i);
        frc::SmartDashboard::PutNumber("Drive D Gain", m_d);
        frc::SmartDashboard::PutNumber("Drive Feed Forward", m_ff);
        frc::SmartDashboard::PutNumber("Drive Max Output", m_max);
        frc::SmartDashboard::PutNumber("Drive Min Output", m_min);
    }

    void LoadFromNetworkTable(TalonFX& driveMotor)
    {
        // Retrieving drive PID values from SmartDashboard
        double p = frc::SmartDashboard::GetNumber("Drive P Gain", 0.0);
        //double i = frc::SmartDashboard::GetNumber("Drive I Gain", 0.0);
        double d = frc::SmartDashboard::GetNumber("Drive D Gain", 0.0);
        double ff = frc::SmartDashboard::GetNumber("Drive Feed Forward", 0.0);
        double max = frc::SmartDashboard::GetNumber("Drive Max Output", 0.0);
        double min = frc::SmartDashboard::GetNumber("Drive Min Output", 0.0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != m_p)) { driveMotor.Config_kP(0, p); m_p = p; }
        //if ((i != m_i)) { driveMotor.Config_kI(0, i); m_i = i; }
        if ((d != m_d)) { driveMotor.Config_kD(0, d); m_d = d; }
        if ((ff != m_ff)) { driveMotor.Config_kF(0, ff); m_ff = ff; }
        
        if ((max != m_max) || (min != m_min))
        { 
            m_min = min;
            m_max = max; 
            driveMotor.ConfigPeakOutputForward(m_max);
            driveMotor.ConfigPeakOutputReverse(m_min);
        }

        SmartDashboard::PutNumber("Drive P Gain", p);
        SmartDashboard::PutNumber("Drive D Gain", d);
        frc::SmartDashboard::GetNumber("Drive Feed Forward", ff);
    }
};

// For each enum here, add a string to c_headerNamesSwerveModule
// and a line like this: 
//      m_logData[ESwerveModuleLogData::e???] = ???;
// to SwerveModule::Periodic
enum class ESwerveModuleLogData : int
{
      eFirstInt
    , eLastInt = eFirstInt

    , eFirstDouble
    , eDesiredAngle = eFirstDouble
    , eTurnEncVolts
    , eTurnEncAngle
    , eMinTurnRads
    , eTurnNeoPidRefPos
    , eTurnNeoEncoderPos
    , eTurnOutputDutyCyc
    , eDrivePidRefSpeed
    , eDriveEncVelocity
    , eDriveOutputDutyCyc
    , eLastDouble
};

const std::vector<std::string> c_headerNamesSwerveModule
{
      "desiredAngle"
    , "turnEncVolts"
    , "turnEncAngle"
    , "minTurnRads"
    , "turnNeoPidRefPos"
    , "turnNeoEncoderPos"
    , "turnOutputDutyCyc"
    , "drivePidRefSpeed"
    , "driveEncVelocity"
    , "driveOutputDutyCyc"
};

class SwerveModule
{
    using radians_per_second_squared_t = compound_unit<radians, inverse<squared<second>>>;

public:
    SwerveModule( int driveMotorChannel
                , int turningMotorChannel
                , GetPulseWidthCallback pulseWidthCallback
                , CANifier::PWMChannel pwmChannel
                , bool driveEncoderReversed
                , double offSet
                , const std::string& name
                , Logger& log);

    frc::SwerveModuleState GetState();

    void Periodic();

    void SetDesiredState(frc::SwerveModuleState &state);

    void ResetEncoders();

    void ResetLog() { m_logData.ResetHeaderLogged(); }

    // Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
    static double ZeroTo2PiRads(double theta);

    // Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
    static double NegPiToPiRads(double theta);

    // Used to confirm the encoder and motor direction are in sync
    // units::volt_t m_tempVoltage = 0.1_V; 
    // void TemporaryRunTurnMotor()
    // {
    //     m_turningMotor.SetVoltage(m_tempVoltage);
    //     m_tempVoltage += 0.1_V; 
    //     if ( m_tempVoltage > 5.0_V)
    //     {
    //          m_tempVoltage = 0.1_V;
    //     }
    // }

private:
    void EncoderToRadians();

    // Determine the smallest magnitude delta angle that can be added to initial angle that will 
    // result in an angle equivalent (but not necessarily equal) to final angle. 
    // All angles in radians
    double MinTurnRads(double init, double final, bool& bOutputReverse);
    meters_per_second_t CalcMetersPerSec();
    double CalcTicksPer100Ms(meters_per_second_t speed);

    // We have to use meters here instead of radians due to the fact that
    // ProfiledPIDController's constraints only take in meters per second and
    // meters per second squared.
    //static constexpr radians_per_second_t kModuleMaxAngularVelocity = radians_per_second_t(wpi::math::pi);                                           // radians per second
    //static constexpr unit_t<radians_per_second_squared_t> kModuleMaxAngularAcceleration = unit_t<radians_per_second_squared_t>(wpi::math::pi * 2.0); // radians per second squared

    double m_offset;
    std::string m_name;
    double m_absAngle = 0.0;

    TalonFX m_driveMotor;
    CANSparkMax m_turningMotor;

    DrivePidParams   m_drivePidParams;
    TurnPidParams   m_turnPidParams;

    CANEncoder m_turnRelativeEncoder = m_turningMotor.GetAlternateEncoder(CANEncoder::AlternateEncoderType::kQuadrature, 
                                                                          ModuleConstants::kTurnEncoderCPR);
    CANPIDController m_turnPIDController = m_turningMotor.GetPIDController();
    GetPulseWidthCallback m_pulseWidthCallback;
    CANifier::PWMChannel m_pwmChannel;

    Timer m_timer;

    using LogData = LogDataT<ESwerveModuleLogData>;
    LogData m_logData;
    Logger& m_log;
};
