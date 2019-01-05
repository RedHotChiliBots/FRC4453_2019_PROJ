package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ChassisShiftHigh extends InstantCommand {

    public ChassisShiftHigh() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.chassis);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        Robot.chassis.shift(true);
    }

}
