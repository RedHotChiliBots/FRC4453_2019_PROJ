package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Wings extends Subsystem {
    private final DoubleSolenoid lift = new DoubleSolenoid(RobotMap.LIFTER_UP_SOLENOID, RobotMap.LIFTER_DOWN_SOLENOID);

    public Wings() {
        lift(); // Initialize Lifter to Up position
    }

    public void lift() {
        lift.set(Value.kForward);
    }

    public void drop() {
        lift.set(Value.kReverse);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
