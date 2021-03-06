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
import edu.wpi.first.wpilibj.GenericHID;

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

  public static int leds = 11;

  // lifts
  public static int liftMotor = 5;

  // cargo grabber
  public static int cargoGrabberMotorL = 10;
  public static int cargoGrabberMotorR = 9;

  // ============= SOLENOIDS (dio) =============

  // panel grabber
  public static final int PanelGrabberGripSolenoid = 0;
  public static final int PanelGrabberReleaseSolenoid = 1;

  // climber solenoids
  public static final int ClimberFrontUpSolenoid = 2;
  public static final int ClimberFrontDownSolenoid = 3;

  public static final int ClimberBackUpSolenoid = 5;
  public static final int ClimberBackDownSolenoid = 6;

  public static final DoubleSolenoid.Value ClimberUp = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value ClimberDown = DoubleSolenoid.Value.kReverse;

  public static final DoubleSolenoid.Value PanelRelease = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value PanelGrab = DoubleSolenoid.Value.kReverse;

  public static final GenericHID.RumbleType leftRumble = GenericHID.RumbleType.kLeftRumble;
  public static final GenericHID.RumbleType rightRumble = GenericHID.RumbleType.kRightRumble;

  // ============= SENSORS (analog) =============

  // chassis pressure sensors
  public static final int highPressureSensor = 0;
  public static final int lowPressureSensor = 1;

  // climber analog dist sensors
  public static final int ClimbFrontDistanceSensor = 2;
  public static final int ClimbBackDistanceSensor = 3;

  // Lift DIstance Sensors
  public static final int panelDistSensor = 4;
  public static final int cargoDistSensor = 6;

  // Servos - pwm slots
  public static final int cargoServo = 0;
  public static final int panelServo = 1;

  // public static final double distFromWall = 10;

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

  public static final int
  // button board
  BTN_1 = 1, BTN_2 = 2, BTN_3 = 3, BTN_4 = 4, BTN_5 = 5, BTN_6 = 6, BTN_7 = 7, BTN_8 = 8, BTN_9 = 9, BTN_10 = 10,
      BTN_11 = 11, BTN_12 = 12, BTN_13 = 13, BTN_14 = 14, BTN_15 = 15, BTN_16 = 16;

  // Attack 3-Axis Joystick Definitions
  public static final int
  // attack 3 axis
  X_AXIS = 0, Y_AXIS = 1, THROTTLE_AXIS = 2;

  public static final int
  // attack 3 buttons
  TRIGGER_1 = 1, BUTTON_2 = 2, BUTTON_3 = 3, BUTTON_4 = 4, BUTTON_5 = 5, BUTTON_6 = 6, BUTTON_7 = 7, BUTTON_8 = 8,
      BUTTON_9 = 9, BUTTON_10 = 10, BUTTON_11 = 11;

  // ============= CONSTANTS =============

  public final static int kTimeoutMs = 30;
  public final static double kNeutralDeadband = 0.001;
  public final static int PID_PRIMARY = 0;
  public final static int SLOT_0 = 0;
  public final static int SLOT_1 = 1;
  public final static int kSlot_Position = SLOT_0;
  public final static int kSlot_Velocity = SLOT_1;

  public static enum DIR {
    LEFT, RIGHT, CENTER
  }

  public static enum MODE {
    PANEL, CARGO
  }

  public static enum LEVEL {
    LEVEL1, LEVEL2, LEVEL3, LOADINGSTATION, SHIP // , SELECTED, CURRENT, PANELLOAD
  }

  public static enum ACTION {
    GRAB, RELEASE
  }

  public static enum KID {
    ON, OFF
  }

  // Heights are inches above base of 19" (Level 1)
  public final static Map<MODE, Map<LEVEL, Double>> height = new HashMap<MODE, Map<LEVEL, Double>>() {
    private static final long serialVersionUID = 1L;
    {
      put(MODE.PANEL, new HashMap<LEVEL, Double>() {
        private static final long serialVersionUID = 1L;
        {
          put(LEVEL.LEVEL3, 56.0); // Score Panel in Rocket Level 3
          put(LEVEL.LEVEL2, 28.0); // Score Panel in Rocket Level 2
          put(LEVEL.LOADINGSTATION, 16.0); // Load Panel
          put(LEVEL.SHIP, 10.5); // Score Panel in Ship
          put(LEVEL.LEVEL1, 0.0); // Score Panel in Rocket Level 1
        }
      });
      put(MODE.CARGO, new HashMap<LEVEL, Double>() {
        private static final long serialVersionUID = 1L;
        {
          put(LEVEL.LEVEL3, 56.0); // Score Cargo in Rocket Level 3
          put(LEVEL.LEVEL2, 28.0); // Score Cargo in Rocket Level 2
          put(LEVEL.LOADINGSTATION, 16.0); // Load Cargo
          put(LEVEL.SHIP, 15.5); // Score Cargo in Shipw
          put(LEVEL.LEVEL1, 0.0); // Score Cargo in Rocket Level 1
        }
      });
    }
  };

}
