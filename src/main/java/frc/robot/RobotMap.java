/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // ============= MOTORS =============

  // chassis
  public static int frontLeftMotor = 1;
  public static int frontRightMotor = 2;
  public static int backLeftMotor = 3;
  public static int backRightMotor = 4;

  // lifts
  public static int upperLiftMotor1 = 5;
  public static int upperLiftMotor2 = 6;
  public static int lowerLiftMotor1 = 7;
  public static int lowerLiftMotor2 = 8;

  // cargo grabber
  public static int cargoGrabberMotor1 = 9;
  public static int cargoGrabberMotor2 = 10;

  // ============= SOLENOIDS (dio) =============

  // panel grabber
  public static final int PanelGrabberGripSolenoid = 0;
  public static final int PanelGrabberReleaseSolenoid = 1;

  // climber solenoids
  public static final int ClimberFrontUpSolenoid = 2;
  public static final int ClimberFrontDownSolenoid = 3;

  public static final int ClimberBackUpSolenoid = 4;
  public static final int ClimberBackDownSolenoid = 5;

  public static final DoubleSolenoid.Value ClimberUp = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value ClimberDown = DoubleSolenoid.Value.kReverse;

  public static final DoubleSolenoid.Value PanelRelease = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value PanelGrab = DoubleSolenoid.Value.kReverse;

  // ============= SENSORS (analog) =============

  // chassis pressure sensors
  public static final int highPressureSensor = 0;
  public static final int lowPressureSensor = 1;

  // climber analog dist sensors
  public static final int ClimbFrontDistanceSensor = 2;
  public static final int ClimbBackDistanceSensor = 3;

  // ============= OI CONTROLLERS =============

  // X-Box Controller Definitions
  public static final int
  // xbox controller axis
  LEFT_X_AXIS = 0, LEFT_Y_AXIS = 1, LEFT_TRIGGER_AXIS = 2, RIGHT_TRIGGER_AXIS = 3, RIGHT_X_AXIS = 4, RIGHT_Y_AXIS = 5,
      DPAD_X_AXIS = 6, DPAD_Y_AXIS = 7;

  public static final int
  // xbox controller buttons
  A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4, LEFT_BUMPER = 5, RIGHT_BUMPER = 6, BACK = 7, START = 8,
      LEFT_STICK = 9, RIGHT_STICK = 10;

  // Attack 3-Axis Joystick Definitions

  public static final int
  // attack 3 axis
  X_AXIS = 0, Y_AXIS = 1, THROTTLE_AXIS = 2;

  public static final int
  // attack 3 buttons
  TRIGGER_1 = 1, BUTTON_2 = 2, BUTTON_3 = 3, BUTTON_4 = 4, BUTTON_5 = 5, BUTTON_6 = 6, BUTTON_7 = 7, BUTTON_8 = 8,
      BUTTON_9 = 9, BUTTON_10 = 10, BUTTON_11 = 11;

  public enum MODE {
    PANEL, CARGO
  }

  public static enum LEVEL {
    LEVEL1, LEVEL2, LEVEL3, LOADINGSTATION, SHIP, SELECTED, CURRENT
  }

  public static LEVEL level;

  public static void setLevel(LEVEL l) {
    level = l;
  }

  public static enum LIFT {
    UPPER, LOWER
  }

  // Heights are inches above base of 19" (Level 1)
  public final static Map<LIFT, Map<LEVEL, Double>> height = new HashMap<LIFT, Map<LEVEL, Double>>() {
    private static final long serialVersionUID = 1L;

    {
      put(LIFT.UPPER, new HashMap<LEVEL, Double>() {
        private static final long serialVersionUID = 1L;

        {
          put(LEVEL.LEVEL3, 28.0); // Score Panel & Cargo in Ship & Rocket
          put(LEVEL.LEVEL2, 28.0); // Score Panel & Cargo in Ship & Rocket
          put(LEVEL.LOADINGSTATION, 16.0); // Load Cargo
          put(LEVEL.SHIP, 10.5); // Score Cargo in Ship
          put(LEVEL.LEVEL1, 0.0); // Load Panel; Score Panel in Ship & Rocket
        }
      });
      put(LIFT.LOWER, new HashMap<LEVEL, Double>() {
        private static final long serialVersionUID = 1L;

        {
          put(LEVEL.LEVEL3, 28.0); // Score Panel & Cargo in Ship & Rocket
          put(LEVEL.LEVEL2, 0.0); // Score Panel & Cargo in Ship & Rocket
          put(LEVEL.LOADINGSTATION, 0.0); // Load Cargo
          put(LEVEL.SHIP, 0.0); // Score Cargo in Ship
          put(LEVEL.LEVEL1, 0.0); // Load Panel; Score Panel in Ship & Rocket
        }
      });
    }
  };

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

  public static void ConfigMotionMagic(WPI_TalonSRX motor1, WPI_TalonSRX motor2, Gains kGains_Distance,
      Gains kGains_Turning) {

    /**
     * Feedback Sensor Configuration
     * 
     * @return
     */
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);

    /* Configure the left Talon's selected sensor as local QuadEncoder */
    motor2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
        RobotMap.PID_PRIMARY, // PID Slot for Source [0, 1]
        RobotMap.kTimeoutMs); // Configuration Timeout

    /*
     * Configure the Remote Talon's selected sensor as a remote sensor for the right
     * Talon
     */
    motor1.configRemoteFeedbackFilter(motor1.getDeviceID(), // Device ID of Source
        RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
        RobotMap.REMOTE_0, // Source number [0, 1]
        RobotMap.kTimeoutMs); // Configuration Timeout

    /* Setup Sum signal to be used for Distance */
    motor1.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.kTimeoutMs); // Feedback Device of
                                                                                                 // Remote Talon
    motor1.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, RobotMap.kTimeoutMs); // Quadurature Encoder of
                                                                                               // current Talon

    /* Setup Difference signal to be used for Turn */
    motor1.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, RobotMap.kTimeoutMs);
    motor1.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, RobotMap.kTimeoutMs);

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
    motor1.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PID_PRIMARY, RobotMap.kTimeoutMs);

    /* Scale Feedback by 0.5 to half the sum of Distance */
    motor1.configSelectedFeedbackCoefficient(0.5, // Coefficient
        RobotMap.PID_PRIMARY, // PID Slot of Source
        RobotMap.kTimeoutMs); // Configuration Timeout

    /*
     * Configure Difference [Difference between both QuadEncoders] to be used for
     * Auxiliary PID Index
     */
    motor1.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, RobotMap.PID_TURN, RobotMap.kTimeoutMs);

    /* Scale the Feedback Sensor using a coefficient */
    motor1.configSelectedFeedbackCoefficient(1, RobotMap.PID_TURN, RobotMap.kTimeoutMs);

    /* Configure output and sensor direction */
    motor1.setInverted(false);
    motor1.setSensorPhase(true);
    motor2.setInverted(false);
    motor2.setSensorPhase(true);

    /* Set status frame periods to ensure we don't have stale data */
    motor1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.kTimeoutMs);
    motor1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.kTimeoutMs);
    motor1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.kTimeoutMs);
    motor1.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.kTimeoutMs);
    motor2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.kTimeoutMs);

    /* Configure neutral deadband */
    motor1.configNeutralDeadband(RobotMap.kNeutralDeadband, RobotMap.kTimeoutMs);
    motor2.configNeutralDeadband(RobotMap.kNeutralDeadband, RobotMap.kTimeoutMs);

    /* Motion Magic Configurations */
    motor1.configMotionAcceleration(TICKS_PER_100MS_PER_SEC, RobotMap.kTimeoutMs);
    motor1.configMotionCruiseVelocity(TICKS_PER_100MS, RobotMap.kTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    motor1.configPeakOutputForward(+1.0, RobotMap.kTimeoutMs);
    motor1.configPeakOutputReverse(-1.0, RobotMap.kTimeoutMs);
    motor2.configPeakOutputForward(+1.0, RobotMap.kTimeoutMs);
    motor2.configPeakOutputReverse(-1.0, RobotMap.kTimeoutMs);

    /* FPID Gains for distance servo */
    motor1.config_kP(RobotMap.kSlot_Distance, kGains_Distance.kP, RobotMap.kTimeoutMs);
    motor1.config_kI(RobotMap.kSlot_Distance, kGains_Distance.kI, RobotMap.kTimeoutMs);
    motor1.config_kD(RobotMap.kSlot_Distance, kGains_Distance.kD, RobotMap.kTimeoutMs);
    motor1.config_kF(RobotMap.kSlot_Distance, kGains_Distance.kF, RobotMap.kTimeoutMs);
    motor1.config_IntegralZone(RobotMap.kSlot_Distance, kGains_Distance.kIzone, RobotMap.kTimeoutMs);
    motor1.configClosedLoopPeakOutput(RobotMap.kSlot_Distance, kGains_Distance.kPeakOutput, RobotMap.kTimeoutMs);
    motor1.configAllowableClosedloopError(RobotMap.kSlot_Distance, 0, RobotMap.kTimeoutMs);

    /* FPID Gains for turn servo */
    motor1.config_kP(RobotMap.kSlot_Turning, kGains_Turning.kP, RobotMap.kTimeoutMs);
    motor1.config_kI(RobotMap.kSlot_Turning, kGains_Turning.kI, RobotMap.kTimeoutMs);
    motor1.config_kD(RobotMap.kSlot_Turning, kGains_Turning.kD, RobotMap.kTimeoutMs);
    motor1.config_kF(RobotMap.kSlot_Turning, kGains_Turning.kF, RobotMap.kTimeoutMs);
    motor1.config_IntegralZone(RobotMap.kSlot_Turning, (int) kGains_Turning.kIzone, RobotMap.kTimeoutMs);
    motor1.configClosedLoopPeakOutput(RobotMap.kSlot_Turning, kGains_Turning.kPeakOutput, RobotMap.kTimeoutMs);
    motor1.configAllowableClosedloopError(RobotMap.kSlot_Turning, 0, RobotMap.kTimeoutMs);

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    motor1.configClosedLoopPeriod(0, closedLoopTimeMs, RobotMap.kTimeoutMs);
    motor1.configClosedLoopPeriod(1, closedLoopTimeMs, RobotMap.kTimeoutMs);

    /**
     * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
     * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
     * local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    motor1.configAuxPIDPolarity(false, RobotMap.kTimeoutMs);

    /* Initialize */
    motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
    zeroSensors(motor1, motor2);

    motor1.selectProfileSlot(RobotMap.kSlot_Distance, RobotMap.PID_PRIMARY);
    motor1.selectProfileSlot(RobotMap.kSlot_Turning, RobotMap.PID_TURN);
  }

  public static void zeroSensors(WPI_TalonSRX motor1, WPI_TalonSRX motor2) {
    motor1.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    motor2.getSensorCollection().setQuadraturePosition(0, RobotMap.kTimeoutMs);
    System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
  }

}
