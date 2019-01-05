package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class GrabberRelease extends InstantCommand {

    public GrabberRelease() {
        super();
        requires(Robot.grabber);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        Robot.grabber.release();
    }

}
