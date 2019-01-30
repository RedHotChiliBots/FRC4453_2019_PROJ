/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LowerLiftInitMotor extends Command {

  private WPI_TalonSRX motor = null;
  private double pos = 0.0;

  public LowerLiftInitMotor(WPI_TalonSRX motor, double pos) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
//    requires(Robot.lLift);
    this.motor = motor;
    this.pos = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    motor.set(ControlMode.Position, 0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lLift.setPosMotor(motor, pos);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   return (Math.abs(motor.getClosedLoopError()) < Robot.prefs.getDouble("LiftPosError", 5.0));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lLift.resetPosMotor(motor, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.lLift.stopMotor(motor);
  }
}
