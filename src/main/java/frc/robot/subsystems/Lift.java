/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CANSparkMaxSendable;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.LEVEL;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import java.util.HashMap;
import java.util.Map;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.Faults;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {

  // Define lift motor
  // public WPI_TalonSRX motor = null;
  public CANSparkMaxSendable motor = null;
  public CANPIDController pid = null;
  public CANEncoder encoder = null;
  public CANDigitalInput fLimit = null;
  public CANDigitalInput rLimit = null;

  // SmartMax does not return target position
  // Saving target position as private varible of Lift subsystem
  // Requires using the setTgtPosition and getTgtPosition methods below
  private double tgtPosition = 0.0;

  // Define lift level
  public LEVEL level = null;

  // Lift Motor, Encoder, Gearbox Calcs
  private static final int COUNTS_PER_REV_MOTOR = 42;
  private static final int GEAR_RATIO = 4;
  private static final int COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEAR_RATIO;
  // Diameter is barrel plus half of rope width
  private static final double BARREL_DIA = 1.5 + (0.125 / 2.0);
  private static final double INCHES_PER_REV = Math.PI * BARREL_DIA;
  public static final int TICKS_PER_INCH = (int) (COUNTS_PER_REV_GEARBOX / INCHES_PER_REV); // Lead screw 1 in/rev

  // Calculate max velocity in tics / 100ms for 600rpm
  private static final double RISE = 78 - 19; // inches
  private static final double TIME = 1.0; // seconds
  private static final double SPEED = RISE / TIME; // inches per second
  private static final double RPM = (SPEED / INCHES_PER_REV) * 60.0; // 28 revs @ 10revs/sec or 600revs/min
  private static final double TICKS_PER_MIN = RPM * COUNTS_PER_REV_GEARBOX;
  private static final int TICKS_PER_100MS = (int) (TICKS_PER_MIN / 60 / 10); // 60sec/min; 10 100ms/sec
  private static final int TICKS_PER_100MS_PER_SEC = (int) (TICKS_PER_100MS * 4.0); // max speed in .25 sec

  public final static Map<String, Double> gainsDistance = new HashMap<String, Double>() {
    private static final long serialVersionUID = 1L;
    {
      put("kCruiseVel", 1.0);
      put("kAccel", 1.0);
      put("kP", 30.0);
      put("kI", 0.01);
      put("kD", 0.0);
      put("kF", 0.0);
      put("kIzone", 100.0);
      put("kPeakOutput", 1.0);
    }
  };

  /**
   * Add your docs here.
   */
  public Lift() {
    super("Lift");

    // Configure lift motor for motion magic
    // motor = new WPI_TalonSRX(RobotMap.liftMotor);
    motor = new CANSparkMaxSendable(RobotMap.liftMotor, CANSparkMax.MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.stopMotor();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setInverted(false);

    pid = motor.getPIDController();
    encoder = motor.getEncoder();
    fLimit = motor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    rLimit = motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

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
    return (motor.getMotorTemperature() * (9.0/5.0))+32;
  }

  /**
   * Lift motor methods in PercentOutput mode
   */
  public void lowerMotor() {
    double spd = -(Robot.prefs.getDouble("LiftDownSpd", 0.25));
    pid.setReference(spd, ControlType.kVoltage, RobotMap.kSlot_Distance);
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  // public void feedMotor() {
  // motor.feed();
  // }

  /**
   * Lift motor methods in MotionMagic mode
   */
  public void initMotorConfig() {
    /* Configure the left Talon's selected sensor as local QuadEncoder */
    //     motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source 
    //         RobotMap.PID_PRIMARY, // PID Slot for Source [0, 1]
    //         RobotMap.kTimeoutMs); // Configuration Timeout

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    //motor.configPeakOutputForward(+1.0, RobotMap.kTimeoutMs); 
    //motor.configPeakOutputReverse(-1.0, RobotMap.kTimeoutMs); 
    pid.setOutputRange(-1.0, 1.0);

    encoder.setPositionConversionFactor(1.0);

    //TODO    pid.setReference(value, ControlType.kSmartMotion, RobotMap.kSlot_Distance);
    pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, RobotMap.kSlot_Distance);

    /* Configure neutral deadband */
    //motor.configNeutralDeadband(RobotMap.kNeutralDeadband, RobotMap.kTimeoutMs);

    motor.setClosedLoopRampRate(0.25);  // Number of seconds to reach full speed

    /* Motion Magic Configurations */
    //motor.configMotionAcceleration(TICKS_PER_100MS_PER_SEC, RobotMap.kTimeoutMs);
    //motor.configMotionCruiseVelocity(TICKS_PER_100MS, RobotMap.kTimeoutMs);
    pid.setSmartMotionMaxAccel(gainsDistance.get("kAccel"), RobotMap.kSlot_Distance);
    pid.setSmartMotionMaxVelocity(gainsDistance.get("kCruiseVel"), RobotMap.kSlot_Distance);

    /* FPID Gains for distance servo */
    //motor.config_kP(RobotMap.kSlot_Distance, gainsDistance.get("kP"), RobotMap.kTimeoutMs);
    //motor.config_kI(RobotMap.kSlot_Distance, gainsDistance.get("kI"), RobotMap.kTimeoutMs);
    //motor.config_kD(RobotMap.kSlot_Distance, gainsDistance.get("kD"), RobotMap.kTimeoutMs);
    //motor.config_kF(RobotMap.kSlot_Distance, gainsDistance.get("kF"), RobotMap.kTimeoutMs);
    //motor.config_IntegralZone(RobotMap.kSlot_Distance, gainsDistance.get("kIzone").intValue(), RobotMap.kTimeoutMs);
    pid.setP(gainsDistance.get("kP"), RobotMap.kSlot_Distance);
    pid.setI(gainsDistance.get("kI"), RobotMap.kSlot_Distance);
    pid.setI(gainsDistance.get("kI"), RobotMap.kSlot_Distance);
    pid.setD(gainsDistance.get("kD"), RobotMap.kSlot_Distance);
    pid.setFF(gainsDistance.get("kF"), RobotMap.kSlot_Distance);
    pid.setIZone(gainsDistance.get("kIZone"), RobotMap.kSlot_Distance);

    //motor.configClosedLoopPeakOutput(RobotMap.kSlot_Distance, gainsDistance.get("kPeakOutput"), RobotMap.kTimeoutMs);
    //motor.configAllowableClosedloopError(RobotMap.kSlot_Distance, 2, RobotMap.kTimeoutMs);
    pid.setSmartMotionAllowedClosedLoopError(2, RobotMap.kSlot_Distance);

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    //    int closedLoopTimeMs = 1;
    //TODO    motor.configClosedLoopPeriod(0, closedLoopTimeMs, RobotMap.kTimeoutMs);
  }

  public void setTgtPosition(double pos) {
    //motor.set(ControlMode.MotionMagic, pos * TICKS_PER_INCH); // , DemandType.AuxPID, target_turn);
    tgtPosition = pos;
    pid.setReference(pos, ControlType.kSmartMotion, RobotMap.kSlot_Distance);
  }

  public double getTgtPosition() {
    return tgtPosition;
  }

  public void resetEncoder(double pos) {
    encoder.setPosition(pos);
    //motor.getSensorCollection().setQuadraturePosition((int) pos * TICKS_PER_INCH, RobotMap.kTimeoutMs);
  }

  public boolean isLimit() {
    //Faults f = new Faults();
    //motor.getFaults();
    //return f.ReverseLimitSwitch;
    return Robot.lift.rLimit.get();
  }
}