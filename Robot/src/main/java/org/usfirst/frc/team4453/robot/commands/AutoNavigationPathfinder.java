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
import org.usfirst.frc.team4453.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutoNavigationPathfinder extends Command {
    
    private static double P = 1.0;
    private static double I = 0.0;
    private static double D = 0.0;

    List<Waypoint> coordinates;

    Trajectory trajectory = null;
    TankModifier tank = null;
    EncoderFollower left, right, center;

    public AutoNavigationPathfinder(List<Waypoint> path) {
        requires(Robot.chassis);
        coordinates = path;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Coordinate start = Robot.navigation.getCurrentCoordinate();
        coordinates.add(0, new Waypoint(start.X, start.Y, start.A));
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, Chassis.MAX_VELOCITY, Chassis.MAX_ACCELERATION, Chassis.MAX_JERK);
        trajectory = Pathfinder.generate(coordinates.toArray(new Waypoint[0]), config);
        tank = new TankModifier(trajectory);
        left = new EncoderFollower(tank.getLeftTrajectory());
        right = new EncoderFollower(tank.getRightTrajectory());

        left.configureEncoder((int)Robot.chassis.getLeftEncoder(), (int)Chassis.CHASSIS_ENCODER_TICKS_PER_REVOLUTION, (int)Chassis.CHASSIS_WHEEL_DIAMETER);
        left.configurePIDVA(P, I, D, 1.0 / Chassis.MAX_VELOCITY, 0.0);
        
        right.configureEncoder((int)Robot.chassis.getRightEncoder(), (int)Chassis.CHASSIS_ENCODER_TICKS_PER_REVOLUTION, (int)Chassis.CHASSIS_WHEEL_DIAMETER);
        right.configurePIDVA(P, I, D, 1.0 / Chassis.MAX_VELOCITY, 0.0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double right_val = right.calculate((int)Robot.chassis.getRightEncoder());
        double left_val = left.calculate((int)Robot.chassis.getLeftEncoder());
        
        double heading = (right.getHeading() + left.getHeading()) / 2.0;

        double turn = 0.8 * (-1.0/80.0) * Pathfinder.boundHalfDegrees(heading - Robot.chassis.getHeading()); // Magic from pathfinder docs.

        Robot.chassis.driveRaw(left_val + turn, right_val - turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return left.isFinished() || right.isFinished() || !Robot.navigation.isOK();
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
