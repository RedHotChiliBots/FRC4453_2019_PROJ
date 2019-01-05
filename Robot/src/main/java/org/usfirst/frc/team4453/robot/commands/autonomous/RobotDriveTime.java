package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class RobotDriveTime extends TimedCommand {

    double speed;
    
    public RobotDriveTime(double s, double time) {
        super(time);
        requires(Robot.chassis);
        speed = s;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.chassis.arcadeDrive(speed, .0);
    }

    protected void execute()
    {
        Robot.chassis.arcadeDrive(speed, .0);
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
