package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;
import org.usfirst.frc.team4453.robot.commands.GrabberGrab;
import org.usfirst.frc.team4453.robot.commands.GrabberThrow;
import org.usfirst.frc.team4453.robot.commands.GrabberTilt;
import org.usfirst.frc.team4453.robot.commands.autonomous.FieldConstants.SwitchPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */ 
public class ApproachAndPlaceCube2 extends CommandGroup {

    private class CenterLeft extends CommandGroup {
        public CenterLeft () {
            addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.LEFT));
            addSequential(new RobotDriveDistance(12.0));
            addSequential(new RobotTurn(-45.368));
            addSequential(new RobotDriveDistance(82.556));
            addSequential(new RobotTurn(45.368));
            addSequential(new RobotDriveDistance(70.0));
            addSequential(new GrabberThrow());
        }
    }

    private class CenterRight extends CommandGroup {
        public CenterRight () {
            addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.RIGHT));
            addSequential(new RobotDriveDistance(12.0));
            addSequential(new RobotTurn(45.368));
            addSequential(new RobotDriveDistance(82.556));
            addSequential(new RobotTurn(-45.368));
            addSequential(new RobotDriveDistance(70.0));
            addSequential(new GrabberThrow());
        }
    }
    
    private class LeftLeft extends CommandGroup {
        public LeftLeft () {
            addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.LEFT));
            addSequential(new RobotDriveDistance(168.0));
            addSequential(new RobotTurn(90.0));
            addSequential(new RobotDriveDistance(42.25));
            addSequential(new GrabberThrow());
        }
    }
    
    private class LeftRight extends CommandGroup {
        public LeftRight () {
            addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.RIGHT));
            addSequential(new RobotDriveDistance(210.0));
            addSequential(new RobotTurn(90.0));
            addSequential(new RobotDriveDistance(177.75));
            addSequential(new RobotTurn(90.0));
            addSequential(new RobotDriveDistance(14.0));
            addSequential(new GrabberThrow());
        }
    }
    private class RightLeft extends CommandGroup {
        public RightLeft () {
            addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.LEFT));
            addSequential(new RobotDriveDistance(210.0));
            addSequential(new RobotTurn(-90.0));
            addSequential(new RobotDriveDistance(177.75));
            addSequential(new RobotTurn(-90.0));
            addSequential(new RobotDriveDistance(14.0));
            addSequential(new GrabberThrow());
        }
    }

    private class RightRight extends CommandGroup {
        public RightRight () {
            addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.RIGHT));
            addSequential(new RobotDriveDistance(168.0));
            addSequential(new RobotTurn(-90.0));
            addSequential(new RobotDriveDistance(42.25));
            addSequential(new GrabberThrow());
        }
    }
    
    public ApproachAndPlaceCube2(Robot.RobotPosition position) {
        requires(Robot.chassis);
        addSequential(new GrabberGrab());
        addSequential(Robot.grabber.new Init());
        addSequential(new TimedCommand(5));
        addSequential(new GrabberTilt(20));
        Robot.chassis.resetNavigation();



        if(position == Robot.RobotPosition.LEFT) {
            Robot.chassis.setXPos(0);
            addParallel(new LeftLeft());
            Robot.chassis.resetNavigation();
            addSequential(new LeftRight());
        }
        else if(position == Robot.RobotPosition.RIGHT) {
            Robot.chassis.setXPos(0);
            addParallel(new RightLeft());
            Robot.chassis.resetNavigation();
            addSequential(new RightRight());
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
