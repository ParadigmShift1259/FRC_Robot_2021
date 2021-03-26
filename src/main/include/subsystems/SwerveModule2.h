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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"

#include <rev\CANSparkMax.h>
#include <rev\CANEncoder.h>

#pragma GCC diagnostic pop

#include <ctre\Phoenix.h>

#include <string>

#include "Constants.h"

// Change this definition to load PID values to both drive and angle
//#define TUNE_MODULE
// Uncomment this to prevent swerve modules from driving
//#define DISABLE_DRIVE

using namespace frc;
using namespace rev;
using namespace units;
using namespace ctre::phoenix::motorcontrol;

using GetPulseWidthCallback = std::function<double (CANifier::PWMChannel)>;

class TurnPidParams2
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
        #ifdef TUNE_MODULE
        frc::SmartDashboard::PutNumber("T_SM_TP", m_p);
        frc::SmartDashboard::PutNumber("T_SM_TI", m_i);
        frc::SmartDashboard::PutNumber("T_SM_TD", m_d);
        frc::SmartDashboard::PutNumber("T_SM_TIZone", m_iz);
        frc::SmartDashboard::PutNumber("T_SM_TFF", m_ff);
        frc::SmartDashboard::PutNumber("T_SM_TMax", m_max);
        frc::SmartDashboard::PutNumber("T_SM_TMin", m_min);
        #endif
    }

    void LoadFromNetworkTable(CANPIDController& turnPIDController)
    {
        double p = frc::SmartDashboard::GetNumber("T_SM_TP", 0.0);
        double i = frc::SmartDashboard::GetNumber("T_SM_TI", 0.0);
        double d = frc::SmartDashboard::GetNumber("T_SM_TD", 0.0);
        double iz = frc::SmartDashboard::GetNumber("T_SM_TIZone", 0.0);
        double ff = frc::SmartDashboard::GetNumber("T_SM_TFF", 0.0);
        double max = frc::SmartDashboard::GetNumber("T_SM_TMax", 0.0);
        double min = frc::SmartDashboard::GetNumber("T_SM_TMin", 0.0);

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

class DrivePidParams2
{
    double m_p = 0.0;
    double m_i = 0.0;
    double m_d = 0.0;
    double m_ff = 0.047619;
    double m_max = 1.0;
    double m_min = -1.0;

public:
   void Load(TalonFX& driveMotor)
    {
        driveMotor.Config_kP(0, m_p);
        driveMotor.Config_kI(0, m_i);
        driveMotor.Config_kD(0, m_d);
        driveMotor.Config_kF(0, m_ff);
        driveMotor.ConfigPeakOutputForward(m_max);
        driveMotor.ConfigPeakOutputReverse(m_min);
        #ifdef TUNE_MODULE
        frc::SmartDashboard::PutNumber("T_SM_DP", m_p);
        frc::SmartDashboard::PutNumber("T_SM_DI", m_i);
        frc::SmartDashboard::PutNumber("T_SM_DD", m_d);
        frc::SmartDashboard::PutNumber("T_SM_DFF", m_ff);
        frc::SmartDashboard::PutNumber("T_SM_DMax", m_max);
        frc::SmartDashboard::PutNumber("T_SM_DMin", m_min);
        #endif
    }

    void LoadFromNetworkTable(TalonFX& driveMotor)
    {
        // Retrieving drive PID values from SmartDashboard
        double p =      frc::SmartDashboard::GetNumber("T_SM_DP", 0.0);
        double i =      frc::SmartDashboard::GetNumber("T_SM_DI", 0);
        double d =      frc::SmartDashboard::GetNumber("T_SM_DD", 0.0);
        double ff =     frc::SmartDashboard::GetNumber("T_SM_DFF", 0.0);
        double max =    frc::SmartDashboard::GetNumber("T_SM_DMax", 0.0);
        double min =    frc::SmartDashboard::GetNumber("T_SM_DMin", 0.0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != m_p)) { driveMotor.Config_kP(0, p); m_p = p; }
        if ((i != m_i)) { driveMotor.Config_kI(0, i); m_i = i; }
        if ((d != m_d)) { driveMotor.Config_kD(0, d); m_d = d; }
        if ((ff != m_ff)) { driveMotor.Config_kF(0, ff); m_ff = ff; }
        
        if ((max != m_max) || (min != m_min))
        {
            driveMotor.ConfigPeakOutputForward(m_max);
            driveMotor.ConfigPeakOutputReverse(m_min);
            m_min = min;
            m_max = max; 
        }
    }
};

class SwerveModule2
{
    using radians_per_second_squared_t = compound_unit<radians, inverse<squared<second>>>;

public:
    SwerveModule2( int driveMotorChannel
                , int turningMotorChannel
                , GetPulseWidthCallback pulseWidthCallback
                , CANifier::PWMChannel pwmChannel
                , bool driveEncoderReversed
                , double offSet
                , const std::string& name);

    frc::SwerveModuleState GetState();

    void Periodic();

    void SetDesiredState(frc::SwerveModuleState &state);

    void ResetEncoders();

    // Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
    static double ZeroTo2PiRads(double theta);

    // Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
    static double NegPiToPiRads(double theta);

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

    DrivePidParams2 m_drivePidParams;
    TurnPidParams2 m_turnPidParams;

    CANEncoder m_turnRelativeEncoder = m_turningMotor.GetAlternateEncoder(CANEncoder::AlternateEncoderType::kQuadrature, 
                                                                          ModuleConstants::kTurnEncoderCPR);
    CANPIDController m_turnPIDController = m_turningMotor.GetPIDController();
    GetPulseWidthCallback m_pulseWidthCallback;
    CANifier::PWMChannel m_pwmChannel;

    Timer m_timer;
};
