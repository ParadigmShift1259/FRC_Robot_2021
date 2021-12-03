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
#include "common/Util.h"

#ifndef Mk2

// Change this definition to load PID values to both drive and angle
#define TUNE_MODULE
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
    double m_ia = 0.0;
    double m_ff = 0.0;
    double m_max = 1.0;
    double m_min = -1.0;

public:
    /// Loads turn PID controller with values, also sends default PID values to SmartDashboard
    /// \param turnPIDController        The CANPIDController of the CANSparkMax responsible for turning
    void Load(CANPIDController& turnPIDController)
    {
        turnPIDController.SetP(m_p);
        turnPIDController.SetI(m_i);
        turnPIDController.SetD(m_d);
        turnPIDController.SetIZone(m_iz);
        turnPIDController.SetIMaxAccum(m_ia);
        turnPIDController.SetFF(m_ff);
        turnPIDController.SetOutputRange(m_min, m_max);
        #ifdef TUNE_MODULE
        frc::SmartDashboard::PutNumber("T_SM_TP", m_p);
        frc::SmartDashboard::PutNumber("T_SM_TI", m_i);
        frc::SmartDashboard::PutNumber("T_SM_TD", m_d);
        frc::SmartDashboard::PutNumber("T_SM_TIZone", m_iz);
        frc::SmartDashboard::PutNumber("T_SM_TIAccum", m_ia);
        frc::SmartDashboard::PutNumber("T_SM_TFF", m_ff);
        frc::SmartDashboard::PutNumber("T_SM_TMax", m_max);
        frc::SmartDashboard::PutNumber("T_SM_TMin", m_min);
        #endif
    }

    /// Loads turn PID controller with on the fly values from SmartDashboard
    /// \param turnPIDController        The CANPIDController of the CANSparkMax responsible for turning
    void LoadFromNetworkTable(CANPIDController& turnPIDController)
    {
        double p = frc::SmartDashboard::GetNumber("T_SM_TP", 0.0);
        double i = frc::SmartDashboard::GetNumber("T_SM_TI", 0.0);
        double d = frc::SmartDashboard::GetNumber("T_SM_TD", 0.0);
        double iz = frc::SmartDashboard::GetNumber("T_SM_TIZone", 0.0);
        double ia = frc::SmartDashboard::GetNumber("T_SM_TIAccum", 0.0);
        double ff = frc::SmartDashboard::GetNumber("T_SM_TFF", 0.0);
        double max = frc::SmartDashboard::GetNumber("T_SM_TMax", 0.0);
        double min = frc::SmartDashboard::GetNumber("T_SM_TMin", 0.0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != m_p)) { turnPIDController.SetP(p); m_p = p; }
        if ((i != m_i)) { turnPIDController.SetI(i); m_i = i; }
        if ((d != m_d)) { turnPIDController.SetD(d); m_d = d; }
        if ((iz != m_iz)) { turnPIDController.SetIZone(iz); m_iz = iz; }
        if ((ia != m_ia)) { turnPIDController.SetIAccum(ia); m_ia = ia; }
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
    double m_p = DriveConstants::kDriveP;
    double m_i = DriveConstants::kDriveI;
    double m_d = DriveConstants::kDriveD;
    double m_ff = 0.047619;
    double m_max = 1.0;
    double m_min = -1.0;

public:
    /// Loads drive PID controller with values, also sends default PID values to SmartDashboard
    /// \param driveMotor        The TalonFX responsible for driving
    void Load(TalonFX& driveMotor)
    {
        driveMotor.Config_kP(0, m_p);
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

    /// Loads drive PID controller with on the fly values from SmartDashboard
    /// \param driveMotor        The TalonFX responsible for driving
    void LoadFromNetworkTable(TalonFX& driveMotor)
    {
        /// Retrieving drive PID values from SmartDashboard
        double p = frc::SmartDashboard::GetNumber("T_SM_DP", 0.0);
        double i = frc::SmartDashboard::GetNumber("T_SM_DI", 0.0);
        double d = frc::SmartDashboard::GetNumber("T_SM_DD", 0.0);
        double ff = frc::SmartDashboard::GetNumber("T_SM_DFF", 0.0);
        double max = frc::SmartDashboard::GetNumber("T_SM_DMax", 0.0);
        double min = frc::SmartDashboard::GetNumber("T_SM_DMin", 0.0);

        /// if PID coefficients on SmartDashboard have changed, write new values to controller
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

    void Periodic();

    /// Get the state for the swerve module pod
    /// \return             The state (vector with speed and angle) representig the current module state
    /// @todo Currently GetState uses the absolute angle instead of the relative angle that we should be using
    frc::SwerveModuleState GetState();

    /// Set the desired state for the swerve module pod
    /// \param state        The state (vector with speed and angle) representing the desired module state
    void SetDesiredState(frc::SwerveModuleState &state);
    /// Resets the drive motor encoders to 0
    void ResetEncoders();
    /// Resync the relative NEO turn encoder to the absolute encoder
    void ResetRelativeToAbsolute();

private:
    /// Calculate the @ref m_absAngle based on the pulse widths from @ref m_pulseWidthCallback
    void EncoderToRadians();
    /// Determine the smallest magnitude delta angle that can be added to initial angle that will 
    /// result in an angle equivalent (but not necessarily equal) to final angle. 
    /// All angles in radians
    /// Currently final - init difference is always chosen regardless of angle
    double MinTurnRads(double init, double final, bool& bOutputReverse);
    /// Calculate the MPS of the drive motor based on its current encoder tick velocity
    /// \return             Meters per second
    meters_per_second_t CalcMetersPerSec();
    /// Calculate drive motor encoder ticks based on the MPS
    /// \param speed        Meters per second
    /// \return             Encoder ticks
    double CalcTicksPer100Ms(meters_per_second_t speed);

    /// The offset, in pulse widths, for syncing the relative encoder to the absolute encoder
    double m_offset;
    /// String used to identify each pod, used for SmartDashboard prints
    std::string m_name;
    /// Absolute angle calculated from the absolute encoder pulse widths
    double m_absAngle = 0.0;

    /// Falon 500 that drives the pod
    TalonFX m_driveMotor;
    /// \name NEO that turns the pod, controls angle with relative encoder and PID
    ///@{
    CANSparkMax m_turningMotor;
    CANEncoder m_turnRelativeEncoder = m_turningMotor.GetAlternateEncoder(CANEncoder::AlternateEncoderType::kQuadrature, 
                                                                          ModuleConstants::kTurnEncoderCPR);
    CANPIDController m_turnPIDController = m_turningMotor.GetPIDController();
    ///@}

    /// PID param loader for the TalonFX
    DrivePidParams2   m_drivePidParams;
    /// PID param loader for the CANSparkMax
    TurnPidParams2   m_turnPidParams;

    /// Callback used to determine the pulse width
    /// \param pwmChannel       Channel to decide which absolute encoder's pulse width values are retrieved
    GetPulseWidthCallback m_pulseWidthCallback;
    CANifier::PWMChannel m_pwmChannel;

    /// Timer used to sync absolute and relative encoders on robot turn on
    Timer m_timer;
};

#endif
