/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Library {

  // Lift Motor, Encoder, Gearbox Calcs
  private static final int COUNTS_PER_REV_MOTOR = 12;
  private static final int GEAR_RATIO = 20;
  private static final int COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEAR_RATIO;
  public static final int TICKS_PER_INCH = COUNTS_PER_REV_GEARBOX; // Lead screw 1 in/rev

  // Calculate max velocity in tics / 100ms for 600rpm
  private static final double RISE = 28; // inches
  private static final double TIME = 2.8; // seconds
  private static final double SPEED = RISE / TIME; // inches per second
  private static final double INCHES_PER_REV = 1.0; // this is the pitch of the lead screw
  private static final double RPM = (SPEED / INCHES_PER_REV) * 60.0; // 28 revs @ 10revs/sec or 600revs/min
  private static final double TICKS_PER_MIN = RPM * COUNTS_PER_REV_GEARBOX;
  private static final int TICKS_PER_100MS = (int) (TICKS_PER_MIN / (60 * 10)); // 60sec/min; 10 100ms/sec
  private static final int TICKS_PER_100MS_PER_SEC = (int) (TICKS_PER_100MS * 4.0); // max speed in .25 sec

  public final static int kSensorUnitsPerRotation = COUNTS_PER_REV_GEARBOX;
  // public final static double kRotationsToTravel = level;
  public final static int kTimeoutMs = 30;
  public final static double kNeutralDeadband = 0.001;

  public final static int PID_PRIMARY = 0;
  public final static int PID_TURN = 1;

  public final static int REMOTE_0 = 0;
  public final static int REMOTE_1 = 1;

  public final static int SLOT_0 = 0;
  public final static int SLOT_1 = 1;
  public final static int SLOT_2 = 2;
  public final static int SLOT_3 = 3;

  public final static int kSlot_Distance = SLOT_0;
  public final static int kSlot_Turning = SLOT_1;
  public final static int kSlot_Velocity = SLOT_2;
  public final static int kSlot_MotProf = SLOT_3;

  public final static Map<String, Double> gainsDistance = new HashMap<String, Double>() {
    private static final long serialVersionUID = 1L;
    {
      put("kCruiseVel", 1.0);
      put("kAccel", 1.0);
      put("kP", 10.0);
      put("kI", 0.0);
      put("kD", 0.0);
      put("kF", 0.0);
      put("kIzone", 100.0);
      put("kPeakOutput", 1.0);
    }
  };

  public final static Map<String, Double> gainsTurning = new HashMap<String, Double>() {
    private static final long serialVersionUID = 1L;
    {
      put("kCruiseVel", 1.0);
      put("kAccel", 1.0);
      put("kP", 10.0);
      put("kI", 0.0);
      put("kD", 0.0);
      put("kF", 0.0);
      put("kIzone", 200.0);
      put("kPeakOutput", 1.0);
    }
  };

  public static void ConfigMotionMagic(WPI_TalonSRX motor1, WPI_TalonSRX motor2) {

    /**
     * Feedback Sensor Configuration
     * 
     * @return
     */
    motor1.configFactoryDefault();
    // motor2.configFactoryDefault();
    motor1.setNeutralMode(NeutralMode.Brake);
    // motor2.setNeutralMode(NeutralMode.Brake);

    /* Configure the left Talon's selected sensor as local QuadEncoder */
    // motor2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local
    // Feedback Source
    // PID_PRIMARY, // PID Slot for Source [0, 1]
    // kTimeoutMs); // Configuration Timeout

    /*
     * Configure the Remote Talon's selected sensor as a remote sensor for the right
     * Talon
     */
    motor1.configRemoteFeedbackFilter(motor1.getDeviceID(), // Device ID of Source
        RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
        REMOTE_0, // Source number [0, 1]
        kTimeoutMs); // Configuration Timeout

    /* Setup Sum signal to be used for Distance */
    motor1.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, kTimeoutMs); // Feedback Device of
                                                                                        // Remote Talon
    motor1.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, kTimeoutMs); // Quadurature Encoder of
                                                                                      // current Talon

    /* Setup Difference signal to be used for Turn */
    motor1.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, kTimeoutMs);
    motor1.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, kTimeoutMs);

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
    motor1.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_PRIMARY, kTimeoutMs);

    /* Scale Feedback by 0.5 to half the sum of Distance */
    motor1.configSelectedFeedbackCoefficient(0.5, // Coefficient
        PID_PRIMARY, // PID Slot of Source
        kTimeoutMs); // Configuration Timeout

    /*
     * Configure Difference [Difference between both QuadEncoders] to be used for
     * Auxiliary PID Index
     */
    motor1.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, PID_TURN, kTimeoutMs);

    /* Scale the Feedback Sensor using a coefficient */
    motor1.configSelectedFeedbackCoefficient(1, PID_TURN, kTimeoutMs);

    /* Configure output and sensor direction */
    motor1.setInverted(false);
    motor1.setSensorPhase(true);
    // motor2.setInverted(false);
    // motor2.setSensorPhase(true);

    /* Set status frame periods to ensure we don't have stale data */
    motor1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
    motor1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, kTimeoutMs);
    motor1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
    motor1.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, kTimeoutMs);
    // motor2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);

    /* Configure neutral deadband */
    motor1.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
    motor2.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

    /* Motion Magic Configurations */
    motor1.configMotionAcceleration(TICKS_PER_100MS_PER_SEC, kTimeoutMs);
    motor1.configMotionCruiseVelocity(TICKS_PER_100MS, kTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    motor1.configPeakOutputForward(+1.0, kTimeoutMs);
    motor1.configPeakOutputReverse(-1.0, kTimeoutMs);
    // motor2.configPeakOutputForward(+1.0, kTimeoutMs);
    // motor2.configPeakOutputReverse(-1.0, kTimeoutMs);

    /* FPID Gains for distance servo */
    motor1.config_kP(kSlot_Distance, gainsDistance.get("kP"), kTimeoutMs);
    motor1.config_kI(kSlot_Distance, gainsDistance.get("kI"), kTimeoutMs);
    motor1.config_kD(kSlot_Distance, gainsDistance.get("kD"), kTimeoutMs);
    motor1.config_kF(kSlot_Distance, gainsDistance.get("kF"), kTimeoutMs);
    motor1.config_IntegralZone(kSlot_Distance, gainsDistance.get("kIzone").intValue(), kTimeoutMs);
    motor1.configClosedLoopPeakOutput(kSlot_Distance, gainsDistance.get("kPeakOutput"), kTimeoutMs);
    motor1.configAllowableClosedloopError(kSlot_Distance, 0, kTimeoutMs);

    /* FPID Gains for turn servo */
    motor1.config_kP(kSlot_Turning, gainsTurning.get("kP"), kTimeoutMs);
    motor1.config_kI(kSlot_Turning, gainsTurning.get("kI"), kTimeoutMs);
    motor1.config_kD(kSlot_Turning, gainsTurning.get("kD"), kTimeoutMs);
    motor1.config_kF(kSlot_Turning, gainsTurning.get("kF"), kTimeoutMs);
    motor1.config_IntegralZone(kSlot_Turning, (int) gainsTurning.get("kIzone").intValue(), kTimeoutMs);
    motor1.configClosedLoopPeakOutput(kSlot_Turning, gainsTurning.get("kPeakOutput"), kTimeoutMs);
    motor1.configAllowableClosedloopError(kSlot_Turning, 0, kTimeoutMs);

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    motor1.configClosedLoopPeriod(0, closedLoopTimeMs, kTimeoutMs);
    motor1.configClosedLoopPeriod(1, closedLoopTimeMs, kTimeoutMs);

    /**
     * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
     * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
     * local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    motor1.configAuxPIDPolarity(false, kTimeoutMs);

    /* Initialize */
    motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
    resetSensors(motor1, motor2, 0.0);

    motor1.selectProfileSlot(kSlot_Distance, PID_PRIMARY);
    motor1.selectProfileSlot(kSlot_Turning, PID_TURN);
  }

  public static void resetSensors(WPI_TalonSRX motor1, WPI_TalonSRX motor2, double pos) {
    motor1.getSensorCollection().setQuadraturePosition((int) pos, kTimeoutMs);
    // motor2.getSensorCollection().setQuadraturePosition((int) pos, kTimeoutMs);
    System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
  }

  public static void setSensorPosition(WPI_TalonSRX motor1, WPI_TalonSRX motor2, double pos) {
    /*
     * Configured for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Quad
     * Encoders' Difference
     */
    // pos - is in target sensor units
    // tartet_turn - is always 0 for our application
    double target_turn = 0.0;
    double target_sensorUnits = pos;
    motor1.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
    // motor2.follow(motor1, FollowerType.AuxOutput1);
  }
}
