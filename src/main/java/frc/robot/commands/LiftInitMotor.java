/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class LiftInitMotor extends Command {

  // private WPI_TalonSRX motor = null;
  private double pos = 0.0;
  private double err = 0.0;

  public LiftInitMotor() {
    requires(Robot.lift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Init LiftInitMotor");

    pos = Robot.prefs.getDouble("LiftMotorReset", 2.0);
    SmartDashboard.putNumber("Lift Motor Reset", pos);
    err = Robot.prefs.getDouble("LiftPosError", 5.0);
    SmartDashboard.putNumber("Lift Pos Error", pos);

    Robot.lift.resetEncoder(pos); // reset both encoder positions, so we can monitor motor2 position
    Robot.lift.setTgtPosition(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lift.motor.feed();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.lift.motor.getClosedLoopError()) < err);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("End LiftInitMotor");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("Interrupt LiftInitMotor");

  }
}
