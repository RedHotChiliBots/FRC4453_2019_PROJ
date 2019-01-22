/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  
  private DoubleSolenoid climbFront; 
  private DoubleSolenoid climbBack; 

  private AnalogInput ClimbLeftDistanceSensor;
	private AnalogInput ClimbRightDistanceSensor;

  public Climber(){
    climbFront = new DoubleSolenoid(RobotMap.ClimberFrontUpSolenoid,RobotMap.ClimberFrontDownSolenoid);
    climbBack = new DoubleSolenoid(RobotMap.ClimberBackUpSolenoid,RobotMap.ClimberBackDownSolenoid);
    ClimbLeftDistanceSensor = new AnalogInput(RobotMap.ClimbLeftDistanceSensor);
    ClimbRightDistanceSensor = new AnalogInput(RobotMap.ClimbRightDistanceSensor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void extendfront(){

  }

  public void extendback(){

  }

  public void retractfront(){

  }

  public void retractback(){

  }

  public void getDistSensor(){

  }
}
