package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.commands.GrabberTilt;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TiltForSwitch extends CommandGroup {
    private final static double ANGLE = 60; // TODO Move constant to dashboard
    
    public TiltForSwitch() {
        addSequential(new GrabberTilt(ANGLE));
    }
}
