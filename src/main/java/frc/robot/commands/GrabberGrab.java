/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.MODE;

public class GrabberGrab extends Command {

  public GrabberGrab() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (Robot.grabber.getMode() == MODE.CARGO) {
      setTimeout(3.0);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.grabber.grabCargoPanel();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean result = false;
    if ((Robot.grabber.getMode() == MODE.CARGO && isTimedOut()) || Robot.grabber.getMode() == MODE.PANEL) {
      result = true;
    }
    return result;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (Robot.grabber.getMode() == MODE.CARGO) {
      Robot.grabber.cargoStop();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    if (Robot.grabber.getMode() == MODE.CARGO) {
      Robot.grabber.cargoStop();
    }
  }
}
