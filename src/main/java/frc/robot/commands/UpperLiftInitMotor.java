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

public class UpperLiftInitMotor extends Command {

  // private WPI_TalonSRX motor = null;
  private double pos = 0.0;

  public UpperLiftInitMotor() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.lLift);
    // this.motor = motor;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Init UpperLiftInitMotor");

    pos = Robot.prefs.getDouble("ULMotorReset", 2.0);
    SmartDashboard.putNumber("ULMotorReset", pos);
    Robot.uLift.resetPosMotor(pos); // reset both encoder positions, so we can monitor motor2 position
    // Robot.lLift.resetPosMotor(motor, pos);
    // Robot.uLift.resetMotorConfig(0.0); // command motor1 to position and motor2
    // to follow
    Robot.uLift.setPosMotor(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.uLift.motor1.feed();
    Robot.uLift.motor2.feed();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.uLift.motor1.getClosedLoopError()) < Robot.prefs.getDouble("LiftPosError", 5.0));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("End UpperLiftInitMotor");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("Interupt UpperLiftInitMotor");
  }
}