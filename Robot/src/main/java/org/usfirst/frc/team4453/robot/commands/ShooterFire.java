package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class ShooterFire extends TimedCommand {

    public ShooterFire() {
        super(3); // TODO: How long to wait?
        requires(Robot.shooter);
    }

    protected void initialize() {
        Robot.shooter.fire();
    }
}
