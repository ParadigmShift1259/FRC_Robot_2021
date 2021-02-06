/**
* This is a very simple robot program that can be used to send telemetry to
* the data_logger script to characterize your drivetrain. If you wish to use
* your actual robot code, you only need to implement the simple logic in the
* autonomousPeriodic function and change the NetworkTables update rate
*/

package dc;

import java.util.function.Supplier;
import java.lang.Math;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// WPI_Talon* imports are needed in case a user has a Pigeon on a Talon
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.AlternateEncoderType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList; 

public class Robot extends TimedRobot {

  static private int ENCODER_EDGES_PER_REV = 1 / 4;
  static private int PIDIDX = 0;
  static private int ENCODER_EPR = 1;
  static private double GEARING = 8.31;
  
  private double encoderConstant = (1 / GEARING);

  Joystick stick;
  DifferentialDrive drive;

  AnalogInput leftFrontE;
  AnalogInput leftBackE;
  AnalogInput rightFrontE;
  AnalogInput rightBackE;

  double leftFrontO = 3.142;
  double leftBackO = 5.963 + 2.67;
  double rightFrontO = 5.105 + 1.831;
  double rightBackO = 0.665 + 0.09 + 1.27;

  CANPIDController leftFrontPID;
  CANPIDController leftBackPID;
  CANPIDController rightFrontPID;
  CANPIDController rightBackPID;

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  String data = "";
  
  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;

  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();
  public Robot() {
    super(.005);
    LiveWindow.disableAllTelemetry();
  }

  public enum Sides {
    LEFT,
    RIGHT,
    FOLLOWER
  }

  // methods to create and setup motors (reduce redundancy)
  public CANSparkMax setupCANSparkMax(int port, Sides side, boolean inverted) {
    // create new motor and set neutral modes (if needed)
    // setup Brushless spark
    CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
    motor.restoreFactoryDefaults(); 
    motor.setIdleMode(IdleMode.kBrake);  
    motor.setInverted(inverted);
    
    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {
    
      
      CANEncoder encoder = motor.getEncoder();



    switch (side) {
      // setup encoder and data collecting methods

      case RIGHT:
        // set right side methods = encoder methods


        rightEncoderPosition = ()
          -> encoder.getPosition() * encoderConstant;
        rightEncoderRate = ()
          -> encoder.getVelocity() * encoderConstant / 60.;

        break;
      case LEFT:
        leftEncoderPosition = ()
          -> encoder.getPosition() * encoderConstant;
        leftEncoderRate = ()
          -> encoder.getVelocity() * encoderConstant / 60.;

        break;
      default:
        // probably do nothing
        break;

      }
    
    }
    

    return motor;

  }

  @Override
  public void robotInit() {
    if (!isReal()) SmartDashboard.putData(new SimEnabler());

    stick = new Joystick(0);
    
    // create left motor
    CANSparkMax leftMotor = setupCANSparkMax(1, Sides.LEFT, false);

    CANSparkMax leftFollowerID7 = setupCANSparkMax(7, Sides.FOLLOWER, false);
    leftFollowerID7.follow(leftMotor, false);

    CANSparkMax rightMotor = setupCANSparkMax(3, Sides.RIGHT, true);
    CANSparkMax rightFollowerID5 = setupCANSparkMax(5, Sides.FOLLOWER, true);
    rightFollowerID5.follow(rightMotor, false);
    drive = new DifferentialDrive(leftMotor, rightMotor);
    drive.setDeadband(0);

    CANSparkMax leftFrontRot = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax leftBackRot = new CANSparkMax(8, MotorType.kBrushless);
    CANSparkMax rightFrontRot = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax rightBackRot = new CANSparkMax(6, MotorType.kBrushless);

    // Custom code to maintain 0 position for rotation SPARK MAXES
    leftFrontPID = new CANPIDController(leftFrontRot);
    leftBackPID = new CANPIDController(leftBackRot);
    rightFrontPID = new CANPIDController(rightFrontRot);
    rightBackPID = new CANPIDController(rightBackRot);

    leftFrontE = new AnalogInput(0);
    leftBackE = new AnalogInput(1);
    rightFrontE = new AnalogInput(2);
    rightBackE = new AnalogInput(3);

    double P = 0.1; // 0.35 // 0.1
    double D = 1; // 1.85 // 1

    leftFrontPID.setP(P);
    leftFrontPID.setD(D);
    leftBackPID.setP(P);
    leftBackPID.setD(D);
    rightFrontPID.setP(P);
    rightFrontPID.setD(D);
    rightBackPID.setP(P);
    rightBackPID.setD(D);

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of WPILib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    // Uncomment for Pigeon
    PigeonIMU pigeon = new PigeonIMU(0);
    gyroAngleRadians = () -> {
      // Allocating a new array every loop is bad but concise
      double[] xyz = new double[3];
      pigeon.getAccumGyro(xyz);
      return Math.toRadians(xyz[2]);
    };

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    drive.tankDrive(0, 0);
    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    System.out.println("Robot disabled");
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data = "";
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {

    double offsetExample = SmartDashboard.getNumber("test", 0);
    double offsetExample2 = SmartDashboard.getNumber("test2", 0);
    double offsetExample3 = SmartDashboard.getNumber("test3", 0);

    double kMaxAV = 4.93;
    double kTurnVTR = 2.0 * Math.PI / kMaxAV;
    
    double leftFrontR = (leftFrontE.getVoltage() * kTurnVTR - leftFrontO - offsetExample3 + 2 * Math.PI) % (2 * Math.PI);
    double leftBackR = (leftBackE.getVoltage() * kTurnVTR - leftBackO - offsetExample2 + 2 * Math.PI) % (2 * Math.PI);
    double rightFrontR = (rightFrontE.getVoltage() * kTurnVTR - rightFrontO + 2 * Math.PI) % (2 * Math.PI);
    double rightBackR = (rightBackE.getVoltage() * kTurnVTR - rightBackO - offsetExample + 2 * Math.PI) % (2 * Math.PI);
    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_front", leftFrontR);
    SmartDashboard.putNumber("l_encoder_back", leftBackR);
    SmartDashboard.putNumber("r_encoder_front", rightFrontR);
    SmartDashboard.putNumber("r_encoder_back", rightBackR);
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
    SmartDashboard.putNumber("test", 0);
    SmartDashboard.putNumber("test2", 0);
    SmartDashboard.putNumber("test3", 0);
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(stick.getY(), -stick.getX());
    
    double offsetExample = SmartDashboard.getNumber("test", 0);
    double offsetExample2 = SmartDashboard.getNumber("test2", 0);
    double offsetExample3 = SmartDashboard.getNumber("test3", 0);

    double kMaxAV = 4.93;
    double kTurnVTR = 2.0 * Math.PI / kMaxAV;

    double leftFrontR = (leftFrontE.getVoltage() * kTurnVTR - leftFrontO - offsetExample3 + 2 * Math.PI) % (2 * Math.PI);
    double leftBackR = (leftBackE.getVoltage() * kTurnVTR - leftBackO - offsetExample2 + 2 * Math.PI) % (2 * Math.PI);
    double rightFrontR = (rightFrontE.getVoltage() * kTurnVTR - rightFrontO + 2 * Math.PI) % (2 * Math.PI);
    double rightBackR = (rightBackE.getVoltage() * kTurnVTR - rightBackO - offsetExample + 2 * Math.PI) % (2 * Math.PI);

    leftFrontPID.setReference(leftFrontR, ControlType.kPosition);
    leftBackPID.setReference(leftBackR, ControlType.kPosition);
    rightFrontPID.setReference(rightFrontR, ControlType.kPosition);
    rightBackPID.setReference(rightBackR, ControlType.kPosition);
  }

  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  /**
  * If you wish to just use your own robot program to use with the data logging
  * program, you only need to copy/paste the logic below into your code and
  * ensure it gets called periodically in autonomous mode
  * 
  * Additionally, you need to set NetworkTables update rate to 10ms using the
  * setUpdateRate call.
  */
  @Override
  public void autonomousPeriodic() {

    double kMaxAV = 4.93;
    double kTurnVTR = 2.0 * Math.PI / kMaxAV;

    double leftFrontR = (leftFrontE.getVoltage() * kTurnVTR - leftFrontO + 2 * Math.PI) % (2 * Math.PI);
    double leftBackR = (leftBackE.getVoltage() * kTurnVTR - leftBackO + 2 * Math.PI) % (2 * Math.PI);
    double rightFrontR = (rightFrontE.getVoltage() * kTurnVTR - rightFrontO + 2 * Math.PI) % (2 * Math.PI);
    double rightBackR = (rightBackE.getVoltage() * kTurnVTR - rightBackO + 2 * Math.PI) % (2 * Math.PI);

    leftFrontPID.setReference(leftFrontR, ControlType.kPosition);
    leftBackPID.setReference(leftBackR, ControlType.kPosition);
    rightFrontPID.setReference(rightFrontR, ControlType.kPosition);
    rightBackPID.setReference(rightBackR, ControlType.kPosition);

    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();

    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive.tankDrive(
      (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
      false
    );

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }
}
