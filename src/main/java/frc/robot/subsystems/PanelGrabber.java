/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class PanelGrabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final DoubleSolenoid grabber; 

  public PanelGrabber(){
    grabber = new DoubleSolenoid(RobotMap.PanelGrabberReleaseSolenoid,RobotMap.PanelGrabberGripSolenoid);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void grab(){

  }

  public void release(){

  }

  public void stop(){
    
  }
}