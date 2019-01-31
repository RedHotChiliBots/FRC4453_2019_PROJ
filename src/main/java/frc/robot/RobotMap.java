/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

  //============= MOTORS =============
  //chassis
  public static int frontLeftMotor = 1;
  public static int frontRightMotor = 2;
  public static int backLeftMotor = 3;
  public static int backRightMotor = 4;

  //lifts
  public static int upperLiftMotor1 = 5;
  public static int upperLiftMotor2 = 6;
  public static int lowerLiftMotor1 = 7;
  public static int lowerLiftMotor2 = 8;
  
  //cargo grabber
  public static int cargoGrabberMotor1 = 9;
  public static int cargoGrabberMotor2 = 10;

  //============= SOLENOIDS (dio) =============
  //panel grabber
  public static final int PanelGrabberGripSolenoid = 0; // TODO
  public static final int PanelGrabberReleaseSolenoid = 1;

//  public static final DoubleSolenoid.Value PanelGrabberGrip = DoubleSolenoid.Value.kForward;
//  public static final DoubleSolenoid.Value PanelGrabberRelease  = DoubleSolenoid.Value.kReverse;

  //climber solenoids
  public static final int ClimberFrontUpSolenoid = 2; // TODO
  public static final int ClimberFrontDownSolenoid = 3;

  public static final int ClimberBackUpSolenoid = 4; // TODO
  public static final int ClimberBackDownSolenoid = 5;

//============= SENSORS (analog) =============
  // chassis pressure sensors
  public static final int highPressureSensor = 0;
  public static final int lowPressureSensor = 1;

  //climber analog dist sensors
  public static final int ClimbFrontDistanceSensor = 2;
  public static final int ClimbBackDistanceSensor = 3;
  
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static final int A_BUTTON	 = 1;
  public static final int B_BUTTON	 = 2;
  public static final int X_BUTTON	 = 3;
  public static final int Y_BUTTON	= 4;
}
