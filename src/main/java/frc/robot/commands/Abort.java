/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Abort extends InstantCommand {
  /**
   * Add your docs here.
   */
  public Abort() {
    super();
    requires(Robot.chassis);
    requires(Robot.climber);
    requires(Robot.grabber);
    requires(Robot.lift);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    DriverStation.reportWarning("Abort!", false);
  }

}
