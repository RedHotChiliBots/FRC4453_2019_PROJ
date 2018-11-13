/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4453.robot.commands;

import java.util.List;

import org.usfirst.frc.team4453.library.NavigationShared.Coordinate;
import org.usfirst.frc.team4453.robot.Robot;
import org.usfirst.frc.team4453.robot.library.Navigation;
import edu.wpi.first.wpilibj.command.Command;

public class AutoNavigation extends Command {

    private List<Coordinate> coordinates;
    private int currentCoordinate = 0;
    private boolean isTurned = false;

    public AutoNavigation(List<Coordinate> coordinates_in) {
        requires(Robot.chassis);

        coordinates = coordinates_in;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

  // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Coordinate goingTo = coordinates.get(currentCoordinate);
        Coordinate currentCoord = Robot.navigation.getCurrentCoordinate();

        if(!isTurned) {
            double angle = Navigation.calculateCoordAngle(currentCoord, goingTo);
            Robot.chassis.turn(angle);
            if(Robot.chassis.angleOnTarget()) {
                isTurned = true;
            }
        } 
        else {
            double distance = Navigation.calculateCoordDist(currentCoord, goingTo);
            Robot.chassis.driveDistance(distance);
            if(Robot.chassis.distanceOnTarget()) {
                currentCoordinate++;
                isTurned = false;
            }
        }

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return currentCoordinate == coordinates.size() || !Robot.navigation.isOK();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.chassis.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.chassis.stop();
    }
}
