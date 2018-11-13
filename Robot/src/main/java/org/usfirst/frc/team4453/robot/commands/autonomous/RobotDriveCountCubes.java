
package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotDriveCountCubes extends Command {
    private static final double SPEED = .25;
    private static final double COMMAND_DELAY = .02; // Senonds between runs of command, 20ms.
    private static final double SAMPLE_PERIOD = .5; // Seconds
    private static final double CUBE_THRESH = 10; // Change in distance that means cube (in)
    
    private int numCubes;
    private int cubesCounted = 0;
    private double[] values = new double[(int) (SAMPLE_PERIOD/COMMAND_DELAY)];
    private boolean waitForRisingEdge = false;
    private boolean isRight;
    public RobotDriveCountCubes(int cubes, boolean right) {
        requires(Robot.chassis);
        numCubes = cubes;
        for(int i = 0; i < values.length; i++)
        {
            values[i] = 0;
        }
        isRight = right;
    }
    
    @Override
    protected void initialize() {
        Robot.chassis.driveWithHeading(SPEED, Robot.ahrs.getYaw());
    }
    
    @Override
    protected void execute() {
        double value = isRight ? Robot.chassis.getRightDistance() : Robot.chassis.getLeftDistance();
        for(int i = 1; i < values.length; i++)
        {
            values[i-1] = values[i];
        }
        values[values.length-1] = value;

        if(values[0] - values[values.length-1] >= CUBE_THRESH) {
            cubesCounted++;
            waitForRisingEdge = true;
        }
        else if(values[values.length-1] - values[0] >= CUBE_THRESH && waitForRisingEdge) {
            waitForRisingEdge = false;
        }
    }
    
    @Override
    protected boolean isFinished() {
        return cubesCounted >= numCubes;
    }
    
    @Override
    protected void end()
    {
        Robot.chassis.stop();
    }
    
    @Override
    protected void interrupted()
    {
        Robot.chassis.stop();
    }
}
