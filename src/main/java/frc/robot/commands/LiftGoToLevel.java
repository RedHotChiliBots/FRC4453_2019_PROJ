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

  private LEVEL base_level = null;
  private LEVEL level = null;

  public LiftGoToLevel() {
    requires(Robot.lift);
    this.base_level = null;
    this.level = null;
  }

  public LiftGoToLevel(LEVEL level) {
    requires(Robot.lift);
    this.base_level = level;
    this.level = level;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (level == null) {
      level = Robot.lift.level;
    }
    Robot.lift.motor.stopMotor();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lift.setTgtPosition(RobotMap.height.get(this.level));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return (Math
    //    .abs(Robot.lift.motor.getSelectedSensorPosition() - Robot.lift.motor.getClosedLoopTarget()) < Robot.prefs
    //        .getDouble("LiftPosError", 5.0));
    return (Math.abs(Robot.lift.encoder.getPosition() - Robot.lift.getTgtPosition()) < 
        Robot.prefs.getDouble("LiftPosError", 5.0));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Go to level finished");
    this.level = base_level;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.level = base_level;
  }
}
