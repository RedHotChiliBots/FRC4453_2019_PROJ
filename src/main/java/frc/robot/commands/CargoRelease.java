/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

public class CargoRelease extends Command {

  private double curr = 0.0;

  public CargoRelease() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.grabber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    curr = Robot.prefs.getDouble("GrabberMotorMinCurrent", 5);
    setTimeout(0.25);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.grabber.cargoRel();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut() && (Robot.grabber.getMotorLCurrent() < curr || Robot.grabber.getMotorLCurrent() < curr);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.grabber.cargoStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.grabber.cargoStop();
  }
}
