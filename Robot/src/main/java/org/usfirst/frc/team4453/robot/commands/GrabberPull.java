package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class GrabberPull extends InstantCommand {

    public GrabberPull() {
        super();
        requires(Robot.grabber);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        Robot.grabber.pull();
    }
}
