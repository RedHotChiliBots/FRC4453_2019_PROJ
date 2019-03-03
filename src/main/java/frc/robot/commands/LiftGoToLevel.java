/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.LEVEL;

public class LiftGoToLevel extends Command {

  private LEVEL level = null;

  public LiftGoToLevel() {
    requires(Robot.lift);
    this.level = null;
  }

  public LiftGoToLevel(LEVEL level) {
    requires(Robot.lift);
    this.level = level;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (level == null){
      level = Robot.lift.level;
    }
    Robot.lift.setTgtPosition(RobotMap.height.get(this.level));
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lift.feedMotor();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.lift.motor.getClosedLoopError()) < Robot.prefs.getDouble("LiftPosError", 5.0));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
