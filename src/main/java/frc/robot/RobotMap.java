/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

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

  public static enum CargoMotor {
    LEFT, RIGHT, CENTER
  }

  public static enum MODE {
    PANEL, CARGO
  }

  public static enum LEVEL {
    LEVEL1, LEVEL2, LEVEL3, LOADINGSTATION, SHIP, SELECTED, CURRENT
  }

  // Heights are inches above base of 19" (Level 1)
  public final static Map<LEVEL, Double> height = new HashMap<LEVEL, Double>() {
    private static final long serialVersionUID = 1L;
    {
      put(LEVEL.LEVEL3, 56.0); // Score Panel & Cargo in Ship & Rocket
      put(LEVEL.LEVEL2, 28.0); // Score Panel & Cargo in Ship & Rocket
      put(LEVEL.LOADINGSTATION, 16.0); // Load Cargo
      put(LEVEL.SHIP, 10.5); // Score Cargo in Ship
      put(LEVEL.LEVEL1, 0.0); // Load Panel; Score Panel in Ship & Rocket
    }
  };

  /*
   * public static enum LIFT { UPPER, LOWER }
   * 
   * // Heights are inches above base of 19" (Level 1) public final static
   * Map<LIFT, Map<LEVEL, Double>> height = new HashMap<LIFT, Map<LEVEL,
   * Double>>() { private static final long serialVersionUID = 1L;
   * 
   * { put(LIFT.UPPER, new HashMap<LEVEL, Double>() { private static final long
   * serialVersionUID = 1L;
   * 
   * { put(LEVEL.LEVEL3, 28.0); // Score Panel & Cargo in Ship & Rocket
   * put(LEVEL.LEVEL2, 28.0); // Score Panel & Cargo in Ship & Rocket
   * put(LEVEL.LOADINGSTATION, 16.0); // Load Cargo put(LEVEL.SHIP, 10.5); //
   * Score Cargo in Ship put(LEVEL.LEVEL1, 0.0); // Load Panel; Score Panel in
   * Ship & Rocket } }); put(LIFT.LOWER, new HashMap<LEVEL, Double>() { private
   * static final long serialVersionUID = 1L;
   * 
   * { put(LEVEL.LEVEL3, 28.0); // Score Panel & Cargo in Ship & Rocket
   * put(LEVEL.LEVEL2, 0.0); // Score Panel & Cargo in Ship & Rocket
   * put(LEVEL.LOADINGSTATION, 0.0); // Load Cargo put(LEVEL.SHIP, 0.0); // Score
   * Cargo in Ship put(LEVEL.LEVEL1, 0.0); // Load Panel; Score Panel in Ship &
   * Rocket } }); } };
   */
}
