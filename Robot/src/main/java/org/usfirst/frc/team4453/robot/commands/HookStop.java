package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class HookStop extends InstantCommand {

    public HookStop() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.hook);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.hook.stop();
    }

}
