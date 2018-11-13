package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoTest extends CommandGroup {

    public AutoTest() {
        requires(Robot.chassis);
//      addSequential(new RobotDriveDistance(12));
        addSequential(new RobotTurn(90));
/*      addSequential(new RobotDriveDistance(12));
        addSequential(new RobotTurn(90));
        addSequential(new RobotDriveDistance(12));
        addSequential(new RobotTurn(90));
        addSequential(new RobotDriveDistance(12));
        addSequential(new RobotTurn(90));
      addSequential(new RobotDriveTime(0.5, 3));
	*/	   }
}
