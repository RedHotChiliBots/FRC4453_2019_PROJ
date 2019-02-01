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

  private DoubleSolenoid climbFront = null; 
  private DoubleSolenoid climbBack = null; 

  private AnalogInput climbFrontDistanceSensor = null;
	private AnalogInput climbBackDistanceSensor = null;

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
    return d;
  }

  public double getDistFrontSensor() {
    return calcDist(climbFrontDistanceSensor.getVoltage());
  }

  public double getDistBackSensor() {
    return calcDist(climbBackDistanceSensor.getVoltage());
  }

  public boolean isFrontClimb() {
    return Robot.chassis.ahrs.getPitch() < Robot.prefs.getDouble("FrontStepAngle", 30.0);
  }

  public boolean isBackClimb() {
    return Robot.chassis.ahrs.getPitch() < Robot.prefs.getDouble("BackStepAngle", 0.0);
  }
  
  public boolean isFrontStep() {
    return isFrontClimb() && getDistFrontSensor() < Robot.prefs.getDouble("FrontStepDist", 2.0);
  }

  public boolean isBackStep() {
    return isBackClimb() && getDistBackSensor() < Robot.prefs.getDouble("BackStepDist", 2.0);
  }
}
