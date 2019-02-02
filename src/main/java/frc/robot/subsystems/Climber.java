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
import frc.robot.Robot;
import frc.robot.RobotMap;
/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DoubleSolenoid climbFront = null; 
  public DoubleSolenoid climbBack = null; 

  public AnalogInput climbFrontDistanceSensor = null;
  public AnalogInput climbBackDistanceSensor = null;

  private Value value = null;

  public Climber() {
    climbFront = new DoubleSolenoid(RobotMap.ClimberFrontUpSolenoid,RobotMap.ClimberFrontDownSolenoid);
    climbBack = new DoubleSolenoid(RobotMap.ClimberBackUpSolenoid,RobotMap.ClimberBackDownSolenoid);
    climbFrontDistanceSensor = new AnalogInput(RobotMap.ClimbFrontDistanceSensor);
    climbBackDistanceSensor = new AnalogInput(RobotMap.ClimbBackDistanceSensor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void extendfront() {
    climbFront.set(Value.kForward);
  }

  public void extendback() {
    climbBack.set(Value.kForward);
  }

  public void retractfront() {
    climbFront.set(Value.kReverse);
  }

  public void retractback() {
    climbFront.set(Value.kReverse);
  }

  private static final double MAXDIST = 30.0; // cm
  private static final double MINDIST = 4.0; // cm
  private static final double MAXVOLT = 2.25; // vdc
  private static final double MINVOLT = 0.4; // vdc
  private static final double DISTRATIO = (MAXDIST-MINDIST)/(MAXVOLT-MINVOLT);

  private double calcDist(double v) {
    double d = 0.0;
    // 30cm = 0.4vdc
    // 4cm = 2.25vcd
    if (v >= MINVOLT && v <= MAXVOLT) {
      d = -(((v - MINVOLT) * DISTRATIO) - MAXDIST);
    }
    return d/2.54;  // return distance in inches
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
    return Robot.chassis.ahrs.getRoll() < Robot.prefs.getDouble("FStepAngleHigh", 18.0)
    && Robot.chassis.ahrs.getRoll() > Robot.prefs.getDouble("FStepAngleLow", 14.0);
  }

  public boolean isBackClimb() {
    return Robot.chassis.ahrs.getRoll() < Robot.prefs.getDouble("BStepAngleHigh", 2.0)
    && Robot.chassis.ahrs.getRoll() > Robot.prefs.getDouble("BStepAngleLow", -2.0);
  }
  
  public boolean isFrontStep() {
    return isFrontClimb() && getDistFrontSensor() < Robot.prefs.getDouble("FrontStepDist", 2.0);
  }

  public boolean isBackStep() {
    return isBackClimb() && getDistBackSensor() < Robot.prefs.getDouble("BackStepDist", 2.0);
  }

  public boolean isStep(AnalogInput distSensor) {
    if(distSensor == climbFrontDistanceSensor){
      return isFrontClimb() && getDistSensor(distSensor) < Robot.prefs.getDouble("FStepDistHigh", 10.0) 
      && getDistSensor(distSensor) > Robot.prefs.getDouble("FStepDistLow", 4.0);
    } else{
      return isBackClimb() && getDistSensor(distSensor) < Robot.prefs.getDouble("BStepDistHigh", 10.0) 
      && getDistSensor(distSensor) > Robot.prefs.getDouble("BStepDistLow", 4.0);
    }
  }

/*  public void SolenoidReverse(DoubleSolenoid solenoid){
    if(solenoid == climbFront){
      solenoid.get();
      if(value == Value.kForward){
        solenoid.set(Value.kReverse);
      }else{
        solenoid.set(Value.kForward);
      }
    }else{
      if(value == Value.kForward){
        solenoid.set(Value.kReverse);
      }else{
        solenoid.set(Value.kForward);
      }
  }*/

  /*public void extend(DoubleSolenoid solenoid){
    solenoid.set(Value.kForward);
  }

  public void retract(DoubleSolenoid solenoid){
    solenoid.set(Value.kReverse);
  }*/

  public void cmdSolenoid(DoubleSolenoid solenoid, Value value){
    solenoid.set(value);
  }
}
