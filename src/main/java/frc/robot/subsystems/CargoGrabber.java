/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CargoGrabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX motor1;
  private WPI_TalonSRX motor2;
  
  
  public CargoGrabber(){
    motor1 = new WPI_TalonSRX(RobotMap.cargoGrabberMotor1);
    motor2 = new WPI_TalonSRX(RobotMap.cargoGrabberMotor2);
  }
  
  @Override
  public void initDefaultCommand() { //stop
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void grab() {
    motor1.set(ControlMode.PercentOutput, -0.5);
    motor2.set(ControlMode.PercentOutput, 0.5);
  }

  public void release() {
    motor1.set(ControlMode.PercentOutput, 0.5);
    motor2.set(ControlMode.PercentOutput, -0.5);
  }

  public void stop(){
    motor1.set(ControlMode.PercentOutput, 0.0);
    motor2.set(ControlMode.PercentOutput, 0.0);
  }

}
