package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class HookRaise extends InstantCommand {

    public HookRaise() {
        super();
        requires(Robot.hook);
    }

    // Called once when the command executes
    protected void initialize() {
        Robot.hook.raise();
    }

}
