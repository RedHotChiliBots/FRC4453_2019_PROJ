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


  public LiftGoToLevel(LEVEL level) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lLift.setPosMotor(Robot.lLift.motor1, RobotMap.height.get(RobotMap.LIFT.LOWER).get(Robot.chassis.level));
    Robot.uLift.setPosMotor(Robot.uLift.motor1, RobotMap.height.get(RobotMap.LIFT.UPPER).get(Robot.chassis.level));
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lLift.motor1.feed();
    Robot.uLift.motor1.feed();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Math.abs(Robot.lLift.motor1.getClosedLoopError()) < Robot.prefs.getDouble("LiftPosError", 5.0)
    && Math.abs(Robot.uLift.motor1.getClosedLoopError()) < Robot.prefs.getDouble("LiftPosError", 5.0)){
      return true;
    }else{
      return false;
    }
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
