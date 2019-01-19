/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


/**
 * Add your docs here.
 */
public class UpperLift extends PIDSubsystem {
  private WPI_TalonSRX motor1;
  private WPI_TalonSRX motor2;
  /**
   * Add your docs here.
   */
  public UpperLift() {
    // Intert a subsystem name and PID values here
    super("SubsystemName", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    motor1 = new WPI_TalonSRX(RobotMap.upperLiftMotor1);
    motor1.set(0.0);
    motor1.setSubsystem("UpperLift");
	  motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    motor2 = new WPI_TalonSRX(RobotMap.upperLiftMotor2);
    motor2.set(0.0);
    motor2.setSubsystem("UpperLift");
    motor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
  }

  public void raise(){

  }

  public void lower(){

  }

  public void stop(){

  }

  public void raiseToPos(){

  }

  public void lowerToPos(){

  }

  public void isLimitSwitchHit(){
    
  }
}
