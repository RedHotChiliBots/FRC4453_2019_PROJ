package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;
import org.usfirst.frc.team4453.robot.commands.ClimberStop;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
    public final int	COUNTS_PER_REV  = 100; // TODO Verify counts per rev
    public final double	INCHES_PER_REV  = 3.3; // TODO Verify inches per rev
    public final double	COUNTS_PER_INCH = COUNTS_PER_REV / INCHES_PER_REV;

    private final WPI_TalonSRX climberLeft  = new WPI_TalonSRX(RobotMap.CLIMBER_RIGHT_MOTOR);
    private final WPI_TalonSRX climberRight = new WPI_TalonSRX(RobotMap.CLIMBER_LEFT_MOTOR);

    public double getDistanceClimbed() {
        return Math.min(climberRight.getSensorCollection().getQuadraturePosition(),
            climberLeft.getSensorCollection().getQuadraturePosition()) / COUNTS_PER_INCH;
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ClimberStop());
    }

    public void resetEncoders() {
        climberLeft.getSensorCollection().setQuadraturePosition(0, 100);
        climberRight.getSensorCollection().setQuadraturePosition(0, 100);
    }

    public void setLeftPower(double speed) {
        climberLeft.set(speed);
    }

    public void setRightPower(double speed) {
        climberRight.set(speed);
    }

    public void stop() {
        climberLeft.set(0);
        climberRight.set(0);
    }
}
