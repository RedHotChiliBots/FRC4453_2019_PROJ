package org.usfirst.frc.team4453.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4453.robot.Robot;

/**
 *
 */
public class RobotDriveToCoord extends CommandGroup {

    private double deltaX;
    private double deltaY;
    private double heading;
    private double angle;
    private double dist;
    
    public RobotDriveToCoord(double x, double y) {
        // Get saved current position
        // Calculate delta between last position and next position
        deltaX = x - Robot.chassis.getXPos();
        deltaY = y - Robot.chassis.getYPos();
        heading = Robot.chassis.getHeading();
        
        System.out.print("dX = " + deltaX + "; dY=" + deltaY + "; h=" + heading);
        
        // Calculate turn angle and drive distance to next position
        dist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        angle = (90 - Math.toDegrees(Math.atan2(deltaY, deltaX))) - heading;
        if (angle>180.0) {
            angle -= 360.0;
        }
        else if (angle<-180.0) {
            angle += 360.0;
        }
        
        System.out.println("dist = " + dist + "; a=" + angle);
        
        // Save next position for next command
        Robot.chassis.setHeading(heading + angle);
        Robot.chassis.setXPos(x);
        Robot.chassis.setYPos(y);
        
        addSequential(new RobotTurn(angle));
        addSequential(new RobotDriveDistance(dist));
    }
}
