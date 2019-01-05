package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimberClimb extends Command {

    static final double	ANGLE_THRESHHOLD = 5;
    static final double	CLIMB_HEIGHT     = 18; // TODO Verify 12in + width of bumpers + extra.

    public ClimberClimb() {
        requires(Robot.climber);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.climber.stop();
        Robot.climber.resetEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        final double roll = Robot.ahrs.getRoll(); // TODO: Pitch/Roll/Yaw?
        if (roll > ANGLE_THRESHHOLD) {
            Robot.climber.setRightPower(Math.sin(Math.toRadians(roll) / 2 + .5));
        }
        else {
            Robot.climber.setRightPower(0.5);
        }
        if (roll < -ANGLE_THRESHHOLD) {
            Robot.climber.setLeftPower(Math.sin(Math.toRadians(roll) / 2 + -0.5));
        }
        else {
            Robot.climber.setLeftPower(-0.5);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.climber.getDistanceClimbed() >= CLIMB_HEIGHT;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.climber.stop();

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.climber.stop();
    }
}
