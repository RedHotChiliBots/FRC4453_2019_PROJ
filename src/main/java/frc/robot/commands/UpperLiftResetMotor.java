/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class UpperLiftResetMotor extends Command {

  private double outputCurrent = 0.0;

  private WPI_TalonSRX motor = null;

  public UpperLiftResetMotor(WPI_TalonSRX motor) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.motor = motor;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.uLift.lowerMotor(motor);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    motor.feed();
    outputCurrent = motor.getOutputCurrent();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (outputCurrent > Robot.prefs.getDouble("CurrentThreshold", 9.0));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lLift.stopMotor(motor);
    Robot.lLift.resetPosMotor(motor, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.lLift.stopMotor(motor);
  }
}
