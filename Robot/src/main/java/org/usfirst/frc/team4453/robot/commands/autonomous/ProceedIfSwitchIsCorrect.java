package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.commands.autonomous.FieldConstants.Alliance;
import org.usfirst.frc.team4453.robot.commands.autonomous.FieldConstants.SwitchPosition;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ProceedIfSwitchIsCorrect extends Command {
    SwitchPosition desired;
    
    public ProceedIfSwitchIsCorrect(SwitchPosition desiredPosition) {
        desired = desiredPosition;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return desired == FieldConstants.getSwitchPosition(Alliance.US);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
