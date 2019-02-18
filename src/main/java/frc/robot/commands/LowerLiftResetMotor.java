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

public class LowerLiftResetMotor extends Command {

  private WPI_TalonSRX motor = null;

  public LowerLiftResetMotor(WPI_TalonSRX motor) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.lLift);
    this.motor = motor;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Init LowerLiftResettMotor");

    Robot.lLift.lowerMotor(motor);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    motor.feed();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (motor.getOutputCurrent() > Robot.prefs.getDouble("CurrentThreshold", 9.0));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lLift.stopMotor(motor);
    Robot.lLift.resetPosMotor(motor, 0.0);
    System.out.println("End LowerLiftResetMotor");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.lLift.stopMotor(motor);
    System.out.println("Interrupt LowerLiftResetMotor");
  }
}
