package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot.RobotPosition;
import org.usfirst.frc.team4453.robot.commands.GrabberRelease;
import org.usfirst.frc.team4453.robot.commands.GrabberThrow;
import org.usfirst.frc.team4453.robot.commands.autonomous.FieldConstants.SwitchPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoAnyPosition extends CommandGroup {

    private class Option1 extends CommandGroup{
        public Option1(RobotPosition position) { 
            addSequential(new ProceedIfSwitchIsCorrect(position == RobotPosition.RIGHT ? SwitchPosition.RIGHT : SwitchPosition.LEFT));
            addSequential(new RobotTurn(position == RobotPosition.RIGHT ? -90 : 90));
            addSequential(new GrabberThrow());
            addSequential(new GrabberRelease());
            addSequential(new RobotTurn(position == RobotPosition.RIGHT ? 90 : -90));
        }
    }

    private class Option2 extends CommandGroup{
        public Option2(RobotPosition position) { 
            addSequential(new ProceedIfSwitchIsCorrect(position == RobotPosition.RIGHT ? SwitchPosition.LEFT : SwitchPosition.RIGHT));
            addSequential(new RobotDriveDistance(FieldConstants.FENCE_DEPTH/2.0 + FieldConstants.ROBOT_LENGTH/2.0 + FieldConstants.CUBE_WIDTH + 5));
            addSequential(new RobotTurn(position == RobotPosition.RIGHT ? -90 : 90));
            addSequential(new RobotDriveCountCubes(5, position == RobotPosition.LEFT));
            addSequential(new RobotTurn(position == RobotPosition.RIGHT ? -90 : 90));
            addSequential(new RobotDriveTime(1, .5));
            addSequential(new GrabberThrow());
            addSequential(new RobotDriveTime(-1, .5));
        }
    }

    public AutoAnyPosition(RobotPosition position) {
        addParallel(new TiltForSwitch());
        addSequential(new RobotDriveDistance(FieldConstants.WALL_TO_FENCE + FieldConstants.FENCE_DEPTH/2.0 - FieldConstants.ROBOT_LENGTH/2.0));
        addParallel(new Option1(position));
        addSequential(new Option2(position));
    }
}
