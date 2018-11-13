package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoCoord extends CommandGroup {

    public AutoCoord() {
        Robot.chassis.resetNavigation();
        requires(Robot.chassis);
        addSequential(new RobotDriveToCoord(0.0,12.0));
        addSequential(new RobotDriveToCoord(12.0,12.0));
        addSequential(new RobotDriveToCoord(12.0,0.0));
        addSequential(new RobotDriveToCoord(0.0,0.0));
    }
}
