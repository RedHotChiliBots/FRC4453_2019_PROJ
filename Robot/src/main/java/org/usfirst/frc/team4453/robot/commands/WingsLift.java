package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class WingsLift extends InstantCommand {

    public WingsLift() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.wings);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.wings.lift();
    }

}
