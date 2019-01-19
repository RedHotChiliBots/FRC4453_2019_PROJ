/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

  //chassis
  public static int frontLeftMotor = 1;
  public static int frontRightMotor = 2;
  public static int backLeftMotor = 3;
  public static int backRightMotor = 0;

  public static final int highPressureSensor = 0;
  public static final int lowPressureSensor = 1;

  public static final int leftDistanceSensor = 2;
  public static final int rightDistanceSensor = 3;

  //lifts
  public static int upperLiftMotor1 = 4;
  public static int upperLiftMotor2 = 4;
  public static int lowerLiftMotor1 = 4;
  public static int lowerLiftMotor2 = 4;
  
  //cargo grabber
  public static int cargoGrabberMotor1 = 4;
  public static int cargoGrabberMotor2 = 4;

  //panel grabber
  public static final int PanelGrabberGripSolenoid = 4; // TODO
  public static final int PanelGrabberReleaseSolenoid = 5; // TODO

  public static final DoubleSolenoid.Value PanelGrabberGrip = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value PanelGrabberRelease  = DoubleSolenoid.Value.kReverse;

  //climber
  public static final int ClimberFrontUpSolenoid = 4; // TODO
  public static final int ClimberFrontDownSolenoid = 5; // TODO

  public static final DoubleSolenoid.Value ClimberFrontUp = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value ClimberFrontDown  = DoubleSolenoid.Value.kReverse;

  public static final int ClimberBackUpSolenoid = 4; // TODO
  public static final int ClimberBackDownSolenoid = 5; // TODO

  public static final DoubleSolenoid.Value ClimberBackUp = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value ClimberBackDown  = DoubleSolenoid.Value.kReverse;

  public static final int ClimbLeftDistanceSensor = 2;
  public static final int ClimbRightDistanceSensor = 3;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
