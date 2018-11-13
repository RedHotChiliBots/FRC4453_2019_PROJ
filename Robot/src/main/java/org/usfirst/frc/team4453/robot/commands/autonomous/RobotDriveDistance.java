package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotDriveDistance extends Command {

    private double distance;
    
    public RobotDriveDistance(double d) {
        //requires(Robot.chassis);
        distance = d;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.chassis.driveDistance(distance);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // return Robot.chassis.distanceOnTarget();
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
	Robot.chassis.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
	Robot.chassis.stop();
    }
}
