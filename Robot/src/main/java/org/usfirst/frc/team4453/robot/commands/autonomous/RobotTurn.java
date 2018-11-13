package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotTurn extends Command {

    private double angle;
    
    public RobotTurn(double a) {
        //requires(Robot.chassis);
        angle = a;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.chassis.turn(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // return Robot.chassis.angleOnTarget();
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
