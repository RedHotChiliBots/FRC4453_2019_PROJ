package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;
import org.usfirst.frc.team4453.robot.commands.autonomous.FieldConstants.SwitchPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */ 
public class ApproachAndPlaceCube extends CommandGroup {

    private class CenterLeft extends CommandGroup {
        public CenterLeft () {
            addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.LEFT));
            addSequential(new RobotDriveToCoord(-58.75, 22.5));
            addSequential(new RobotDriveToCoord(-58.75, 140));
        }
    }
    
    private class CenterRight extends CommandGroup {
        public CenterRight () {
            addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.RIGHT));
            addSequential(new RobotDriveToCoord(58.75, 22.5));
            addSequential(new RobotDriveToCoord(58.75, 140));
        }
    }
    
    public ApproachAndPlaceCube(Robot.RobotPosition position) {
        requires(Robot.chassis);
        addSequential(Robot.grabber.new Init());
        Robot.chassis.resetNavigation();

        if(position == Robot.RobotPosition.LEFT) {
            Robot.chassis.setXPos(-58.75);
            addSequential(new RobotDriveToCoord(-58.75, 140.0));
        }
        else if(position == Robot.RobotPosition.RIGHT) {
            Robot.chassis.setXPos(58.75);
            addSequential(new RobotDriveToCoord(58.75, 140.0));
        }
        else if(position == Robot.RobotPosition.CENTER) {
            Robot.chassis.setXPos(0);
            addParallel(new CenterLeft());
            Robot.chassis.resetNavigation();
            addSequential(new CenterRight());
        }


        // addParallel(new TiltForSwitch());
        // addSequential(new RobotDriveDistance(FieldConstants.AUTO_LINE_DIST + FieldConstants.ROBOT_LENGTH));
        // addSequential(new GrabberTilt(130));
        // addSequential(new GrabberGrab());
        // addSequential(new RobotDriveTime(-0.75, 3.5)); // TODO - Move constants to dashboard
        // addSequential(new ProceedIfSwitchIsCorrect(position == RobotPosition.RIGHT ? SwitchPosition.RIGHT : SwitchPosition.LEFT));
        // addSequential(new RobotDriveDistance(FieldConstants.WALL_TO_FENCE - (FieldConstants.AUTO_LINE_DIST + FieldConstants.ROBOT_LENGTH)));
        // addSequential(new GrabberTilt(20));
        // addSequential(new GrabberThrow());
        // addSequential(new RobotDriveDistance(-FieldConstants.ROBOT_LENGTH));
    }
}
