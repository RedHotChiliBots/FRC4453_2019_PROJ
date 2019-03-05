/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Climber - Manage Climb Pistons
 * 
 * The Climber subsystem provides distance sensors and piston control to allow
 * for autonomous climb to Level 2.
 */
public class Climber extends Subsystem {

  // Front & Back Climb Solenoids
  public DoubleSolenoid climbFront = null;
  public DoubleSolenoid climbBack = null;

  // Front & Back Distance Sensors
  public AnalogInput climbFrontDistanceSensor = null;
  public AnalogInput climbBackDistanceSensor = null;

  public Climber() {
    super("Climber");
    System.out.println("Climber init");

    // Initialize climb solenoids
    climbFront = new DoubleSolenoid(RobotMap.ClimberFrontUpSolenoid, RobotMap.ClimberFrontDownSolenoid);
    climbBack = new DoubleSolenoid(RobotMap.ClimberBackUpSolenoid, RobotMap.ClimberBackDownSolenoid);

    // Initialize distance sensors
    climbFrontDistanceSensor = new AnalogInput(RobotMap.ClimbFrontDistanceSensor);
    climbBackDistanceSensor = new AnalogInput(RobotMap.ClimbBackDistanceSensor);

    retractfront();
    retractback();
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new ClimberRetract());
  }

  public void extendfront() {
    climbFront.set(RobotMap.ClimberUp);
  }

  public void extendback() {
    climbBack.set(RobotMap.ClimberUp);
  }

  public void retractfront() {
    climbFront.set(RobotMap.ClimberDown);
  }

  public void retractback() {
    climbBack.set(RobotMap.ClimberDown);
  }

  private static final double MAXDIST = 30.0; // cm
  private static final double MINDIST = 4.0; // cm
  private static final double MAXVOLT = 2.25; // vdc
  private static final double MINVOLT = 0.4; // vdc
  private static final double DISTRATIO = (MAXDIST - MINDIST) / (MAXVOLT - MINVOLT);

  public double calcDist(double v) {
    double d = 0.0;
    // 30cm = 0.4vdc
    // 4cm = 2.25vcd
    if (v >= MINVOLT && v <= MAXVOLT) {
      d = -(((v - MINVOLT) * DISTRATIO) - MAXDIST);
    }
    return d / 2.54; // return distance in inches
  }

  public double getDistFrontSensor() {
    return calcDist(climbFrontDistanceSensor.getVoltage());
  }

  public double getDistBackSensor() {
    return calcDist(climbBackDistanceSensor.getVoltage());
  }

  public double getDistSensor(AnalogInput distSensor) {
    return calcDist(distSensor.getVoltage());
  }

  public boolean isFrontClimb() {
    return Robot.chassis.getPitch() > -Robot.prefs.getDouble("FStepAngleHigh", 18.0)
        && Robot.chassis.getPitch() < -Robot.prefs.getDouble("FStepAngleLow", 14.0);
  }

  public boolean isBackClimb() {
    return Robot.chassis.getPitch() > -Robot.prefs.getDouble("BStepAngleHigh", 2.0)
        && Robot.chassis.getPitch() < -Robot.prefs.getDouble("BStepAngleLow", -2.0);
  }

  public boolean isFrontStep() {
    return isFrontClimb() && getDistFrontSensor() < Robot.prefs.getDouble("FrontStepDist", 2.0);
  }

  public boolean isBackStep() {
    return isBackClimb() && getDistBackSensor() < Robot.prefs.getDouble("BackStepDist", 2.0);
  }

  // took out loo for angle
  public boolean isStep(AnalogInput distSensor) {
    if (distSensor == climbFrontDistanceSensor) {
      return getDistSensor(distSensor) < Robot.prefs.getDouble("FStepDistHigh", 10.0)
          && getDistSensor(distSensor) > Robot.prefs.getDouble("FStepDistLow", 4.0);
    } else {
      return getDistSensor(distSensor) < Robot.prefs.getDouble("BStepDistHigh", 10.0)
          && getDistSensor(distSensor) > Robot.prefs.getDouble("BStepDistLow", 4.0);
    }
  }

  public void cmdSolenoid(DoubleSolenoid solenoid, Value value) {
    solenoid.set(value);
  }
}
