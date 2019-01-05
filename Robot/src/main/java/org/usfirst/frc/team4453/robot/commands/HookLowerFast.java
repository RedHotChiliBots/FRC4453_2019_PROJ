package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HookLowerFast extends Command {

    public HookLowerFast() {
        super();
        requires(Robot.hook);
    }

    // Called once when the command executes
    protected void initialize() {
        Robot.hook.lowerFast();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
