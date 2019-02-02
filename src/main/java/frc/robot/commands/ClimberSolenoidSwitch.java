/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimberSolenoidSwitch extends Command {

  private DoubleSolenoid solenoid = null;
  private Value value = null;

  public ClimberSolenoidSwitch(DoubleSolenoid solenoid, Value value) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.solenoid = solenoid;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.climber.cmdSolenoid(solenoid, value);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(solenoid == Robot.climber.climbFront){
      return Robot.chassis.ahrs.getPitch() < Robot.prefs.getDouble("FrontStepAngleHigh", 16.0)
      && Robot.chassis.ahrs.getPitch() > Robot.prefs.getDouble("FrontStepAngleLow", 16.0);
    }else{
      return Robot.chassis.ahrs.getPitch() < Robot.prefs.getDouble("BackStepAngleHigh", 0.0)
      && Robot.chassis.ahrs.getPitch() > Robot.prefs.getDouble("BackStepAngleLow", 0.0);
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
