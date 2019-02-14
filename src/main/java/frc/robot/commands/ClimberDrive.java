/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimberDrive extends Command {

  private AnalogInput distSensor = null;

  public ClimberDrive(AnalogInput distSensor) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.distSensor = distSensor;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("init ClimberDrive");
    // Robot.chassis.setFollow();
    Robot.chassis.driveChassis(0.0, 0.45, 0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.chassis.setPos(100);
    Robot.chassis.backleft.feed();
    Robot.chassis.backright.feed();
    Robot.chassis.frontleft.feed();
    Robot.chassis.frontright.feed();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.climber.isStep(distSensor);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("ending ClimberDrive");
    // Robot.chassis.setPercentOut();
    Robot.chassis.driveChassis(0.0, 0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    // Robot.chassis.setPercentOut();
    Robot.chassis.driveChassis(0.0, 0.0, 0.0);
  }
}
