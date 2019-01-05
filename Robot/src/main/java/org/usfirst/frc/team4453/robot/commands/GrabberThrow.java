package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class GrabberThrow extends TimedCommand {

    public GrabberThrow() {
        super(1.5);
        requires(Robot.grabber);
    }
    
    protected void execute() {
        Robot.grabber.toss();
    }

}
