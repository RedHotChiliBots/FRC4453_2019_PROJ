/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CANSparkMaxSendable;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.LEVEL;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {

  // Define lift motor
  public CANSparkMaxSendable motor = null;
  public CANPIDController pid = null;
  public CANEncoder encoder = null;
  public CANDigitalInput rLimit = null;

  // SmartMax does not return target position
  // Saving target position as private varible of Lift subsystem
  // Requires using the setTgtPosition and getTgtPosition methods below
  private double tgtPosition = 0.0;

  // Define lift level
  public LEVEL level = null;

  // Lift Motor, Encoder, Gearbox Calcs
  private static final int GEAR_RATIO = 48;
  // Diameter is barrel plus half of rope width
  private static final double BARREL_DIA = 1.5 + (0.125 / 2.0);

  private static final double INCHES_PER_REV_GEARBOX = Math.PI * BARREL_DIA;
  private static final double INCHES_PER_REV_MOTOR = INCHES_PER_REV_GEARBOX / GEAR_RATIO;

  private static final double REV_PER_INCH_GEARBOX = 1.0 / INCHES_PER_REV_GEARBOX;
  private static final double REV_PER_INCH_MOTOR = 1.0 / INCHES_PER_REV_MOTOR;

  /**
   * Add your docs here.
   */
  public Lift() {
    super("Lift");

    // Configure lift motor for smart motion
    motor = new CANSparkMaxSendable(RobotMap.liftMotor, CANSparkMax.MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.stopMotor();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setInverted(false);

    pid = motor.getPIDController();
    encoder = motor.getEncoder();
    rLimit = motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    rLimit.enableLimitSwitch(true);

    initMotorConfig();

    // Initialize lift to LEVEL1
    level = LEVEL.LEVEL1;

    // Add Sendable data to dashboard
    motor.setSubsystem("UpperLift");
    SmartDashboard.putData("Lift Motor", motor);
  }

  @Override
  public void initDefaultCommand() {
  }

  /**
   * Lift level getters and setters
   */
  public void setLevel(LEVEL l) {
    level = l;
  }

  public LEVEL getLevel() {
    return level;
  }

  public double getMotorCurrent() {
    return motor.getOutputCurrent();
  }

  public double getMotorTemp() {
    return (motor.getMotorTemperature() * (9.0 / 5.0)) + 32;
  }

  /**
   * Lift motor methods in PercentOutput mode
   */
  public void lowerMotor() {
    double spd = -(Robot.prefs.getDouble("LiftDownSpd", 0.10));
    pid.setReference(spd, ControlType.kDutyCycle, RobotMap.kSlot_Velocity);
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  /**
   * Lift motor methods in Smart Motion mode
   */
  public void initMotorConfig() {
    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    pid.setOutputRange(-1.0, 1.0);

    encoder.setPositionConversionFactor(INCHES_PER_REV_MOTOR); // One motor rev equals this
    encoder.setVelocityConversionFactor(1.0 / GEAR_RATIO);

    pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, RobotMap.kSlot_Position);
    pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, RobotMap.kSlot_Velocity);

    /* Configure neutral deadband */
    // motor.configNeutralDeadband(RobotMap.kNeutralDeadband, RobotMap.kTimeoutMs);

    motor.setClosedLoopRampRate(0.25); // Number of seconds to reach full speed

    /* Smart Motion Magic */
    pid.setSmartMotionMaxAccel(560.0 * 4, RobotMap.kSlot_Position);
    pid.setSmartMotionMaxVelocity(560.0, RobotMap.kSlot_Position);

    pid.setSmartMotionMaxAccel(560.0 * 4, RobotMap.kSlot_Velocity);
    pid.setSmartMotionMaxVelocity(900.0, RobotMap.kSlot_Velocity);

    /* FPID Gains for distance servo */
    pid.setP(0.00095, RobotMap.kSlot_Position);
    pid.setI(0.000075, RobotMap.kSlot_Position);
    pid.setD(0.0, RobotMap.kSlot_Position);
    pid.setFF(0.0, RobotMap.kSlot_Position);
    pid.setIZone(0.0625, RobotMap.kSlot_Position);

    pid.setP(0.002, RobotMap.kSlot_Velocity);
    pid.setI(0.002, RobotMap.kSlot_Velocity);
    pid.setD(0.0, RobotMap.kSlot_Velocity);
    pid.setFF(0.0, RobotMap.kSlot_Velocity);
    pid.setIZone(20.0, RobotMap.kSlot_Velocity);

    pid.setSmartMotionAllowedClosedLoopError(0.25, RobotMap.kSlot_Position);
    pid.setSmartMotionAllowedClosedLoopError(2, RobotMap.kSlot_Velocity);
  }

  // Set target position in inches
  public void setTgtPosition(double pos) {
    tgtPosition = pos;
    // pid.setReference(pos * REV_PER_INCH_MOTOR, ControlType.kSmartMotion,
    // RobotMap.kSlot_Position);
    pid.setReference(pos, ControlType.kSmartMotion, RobotMap.kSlot_Position);
  }

  public double getTgtPosition() {
    return tgtPosition;
  }

  public void resetEncoder(double pos) {
    encoder.setPosition(pos);
    // pid.setReference(pos, ControlType.kSmartMotion, RobotMap.kSlot_Position);
    // motor.getSensorCollection().setQuadraturePosition((int) pos * TICKS_PER_INCH,
    // RobotMap.kTimeoutMs);
  }

  public boolean isLimit() {
    return Robot.lift.rLimit.get();
  }
}