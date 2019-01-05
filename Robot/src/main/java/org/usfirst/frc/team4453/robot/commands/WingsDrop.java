package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class WingsDrop extends InstantCommand {

    public WingsDrop() {
        super();
        requires(Robot.wings);
    }

    // Called once when the command executes
    // protected void initialize() {
    // }
    @Override
    protected void execute() {
        if (Timer.getMatchTime() < 30) {
            Robot.wings.drop();
        }
    }
}
