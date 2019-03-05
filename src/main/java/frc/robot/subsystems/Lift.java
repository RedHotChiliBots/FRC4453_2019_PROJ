/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotMap;
import frc.robot.RobotMap.LEVEL;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {

  // Define lift motor
  public WPI_TalonSRX motor = null;

  // Define lift level
  public LEVEL level = null;

  // Lift Motor, Encoder, Gearbox Calcs
  private static final int COUNTS_PER_REV_MOTOR = 12;
  private static final int GEAR_RATIO = 20;
  private static final int COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEAR_RATIO;
  private static final double BARREL_DIA = 1.5;
  private static final double INCHES_PER_REV = Math.PI * BARREL_DIA;
  public static final int TICKS_PER_INCH = (int) (COUNTS_PER_REV_GEARBOX / INCHES_PER_REV); // Lead screw 1 in/rev

  // Calculate max velocity in tics / 100ms for 600rpm
  private static final double RISE = 78 - 19; // inches
  private static final double TIME = 2.8; // seconds
  private static final double SPEED = RISE / TIME; // inches per second
  private static final double RPM = (SPEED / INCHES_PER_REV) * 60.0; // 28 revs @ 10revs/sec or 600revs/min
  private static final double TICKS_PER_MIN = RPM * COUNTS_PER_REV_GEARBOX;
  private static final int TICKS_PER_100MS = (int) (TICKS_PER_MIN / (60 * 10)); // 60sec/min; 10 100ms/sec
  private static final int TICKS_PER_100MS_PER_SEC = (int) (TICKS_PER_100MS * 4.0); // max speed in .25 sec

  public final static Map<String, Double> gainsDistance = new HashMap<String, Double>() {
    private static final long serialVersionUID = 1L;
    {
      put("kCruiseVel", 1.0);
      put("kAccel", 1.0);
      put("kP", 20.0);
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
    motor = new WPI_TalonSRX(RobotMap.liftMotor);
    motor.set(ControlMode.PercentOutput, 0.0);
    motor.setSubsystem("UpperLift");
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);

    initMotorConfig();

    // Initialize lift to LEVEL1
    level = LEVEL.LEVEL1;

    // Add Sendable data to dashboard
    SmartDashboard.putData("Lift Motor", motor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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

  /**
   * Lift motor methods in PercentOutput mode
   */
  public void lowerMotor() {
    motor.set(ControlMode.PercentOutput, -0.35);
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  public void feedMotor() {
    motor.feed();
  }

  /**
   * Lift motor methods in MotionMagic mode
   */
  public void initMotorConfig() {
    /* Configure the left Talon's selected sensor as local QuadEncoder */
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
        RobotMap.PID_PRIMARY, // PID Slot for Source [0, 1]
        RobotMap.kTimeoutMs); // Configuration Timeout

    /* Configure output and sensor direction */
    motor.setInverted(true);
    motor.setSensorPhase(true);

    /* Configure neutral deadband */
    motor.configNeutralDeadband(RobotMap.kNeutralDeadband, RobotMap.kTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    motor.configPeakOutputForward(+1.0, RobotMap.kTimeoutMs);
    motor.configPeakOutputReverse(-1.0, RobotMap.kTimeoutMs);

    /* Motion Magic Configurations */
    motor.configMotionAcceleration(TICKS_PER_100MS_PER_SEC, RobotMap.kTimeoutMs);
    motor.configMotionCruiseVelocity(TICKS_PER_100MS, RobotMap.kTimeoutMs);

    /* FPID Gains for distance servo */
    motor.config_kP(RobotMap.kSlot_Distance, gainsDistance.get("kP"), RobotMap.kTimeoutMs);
    motor.config_kI(RobotMap.kSlot_Distance, gainsDistance.get("kI"), RobotMap.kTimeoutMs);
    motor.config_kD(RobotMap.kSlot_Distance, gainsDistance.get("kD"), RobotMap.kTimeoutMs);
    motor.config_kF(RobotMap.kSlot_Distance, gainsDistance.get("kF"), RobotMap.kTimeoutMs);
    motor.config_IntegralZone(RobotMap.kSlot_Distance, gainsDistance.get("kIzone").intValue(), RobotMap.kTimeoutMs);

    motor.configClosedLoopPeakOutput(RobotMap.kSlot_Distance, gainsDistance.get("kPeakOutput"), RobotMap.kTimeoutMs);
    motor.configAllowableClosedloopError(RobotMap.kSlot_Distance, 2, RobotMap.kTimeoutMs);

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    motor.configClosedLoopPeriod(0, closedLoopTimeMs, RobotMap.kTimeoutMs);
  }

  public void setTgtPosition(double pos) {
    motor.set(ControlMode.MotionMagic, pos * TICKS_PER_INCH); // , DemandType.AuxPID, target_turn);
  }

  public void resetEncoder(double pos) {
    motor.getSensorCollection().setQuadraturePosition((int) pos * TICKS_PER_INCH, RobotMap.kTimeoutMs);
  }

  public boolean isLimit() {
    Faults f = new Faults();
    motor.getFaults(f);
    return f.ReverseLimitSwitch;
  }
}