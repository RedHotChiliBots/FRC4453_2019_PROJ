package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;
import org.usfirst.frc.team4453.robot.commands.GrabberTeleop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Grabber extends Subsystem {
    /**
     * Number of encoder ticks per one degree of tilt.
     */
    private static final int COUNTS_PER_REV_MOTOR   = 12;
    private static final int GEAR_RATIO             = 100;
    private static final int COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEAR_RATIO;
    private static final double TICKS_PER_DEGREE    = COUNTS_PER_REV_GEARBOX / 360.0;

    private static final double MAX_ANGLE = 175; // TODO: Determine mechanically
    
    private static final double kF = 0, kP = 60, kI= 0, kD = 0;
    
    private WPI_TalonSRX   left  = new WPI_TalonSRX(RobotMap.GRABBER_LEFT_MOTOR);
    private WPI_TalonSRX   right = new WPI_TalonSRX(RobotMap.GRABBER_RIGHT_MOTOR);
    private WPI_TalonSRX   tilt  = new WPI_TalonSRX(RobotMap.GRABBER_TILT_MOTOR);
    private DoubleSolenoid grip  = new DoubleSolenoid(RobotMap.GRABBER_GRIP_SOLENOID, RobotMap.GRABBER_RELEASE_SOLENOID);
    
    private boolean isInitialized = false;
 
    /**
     * Command to initialize the Grabber.
     * Sets subsystem to default state and resets encoder zeros.
     */

    public class Init extends CommandGroup {
        public Init() {
            addSequential(new ResetCommand());
        }
    }

    private class ResetCommand extends Command {
        public ResetCommand() {
                setInterruptible(false);
                requires(Grabber.this);
        }

        @Override
        protected void initialize() {
            //System.out.println("Grabber Init initializing...");
            isInitialized=false;
            left.setNeutralMode(NeutralMode.Brake);
            right.setNeutralMode(NeutralMode.Brake);
            left.neutralOutput();
            right.neutralOutput();

            grip.set(Value.kForward);

            tilt.setNeutralMode(NeutralMode.Brake);
            tilt.set(ControlMode.PercentOutput, -.5); // TODO: Verify correct direction?
        }

        protected void execute() {
            //System.out.println("Grabber Init execute");
            tilt.set(ControlMode.PercentOutput, -.5);
        }

        @Override
        protected boolean isFinished() {
            return tilt.getSensorCollection().isRevLimitSwitchClosed(); // TODO Verify correct direction?
        }

        @Override
        protected void end() {
            //System.out.println("Grabber Init end");
            tilt.neutralOutput();
            tilt.getSensorCollection().setQuadraturePosition(0, 100);
            tilt.set(ControlMode.Position, 0);
            isInitialized=true;
        }
    }

    public Grabber() {
        tilt.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
        tilt.setSensorPhase(true);
        tilt.configForwardSoftLimitThreshold((int) (MAX_ANGLE * TICKS_PER_DEGREE), 100);
        tilt.configForwardSoftLimitEnable(false, 100);
        tilt.configClosedloopRamp(0.1, 100);
        tilt.configPeakOutputForward(1, 100);
        tilt.configPeakOutputReverse(-0.5, 100);
        tilt.config_kF(0, kF, 100);
        tilt.config_kP(0, kP, 100);
        tilt.config_kI(0, kI, 100);
        tilt.config_kD(0, kD, 100);
        tilt.selectProfileSlot(0, 0);
    }
    
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new GrabberTeleop());
    }

    public void init() {
        new Init().start();
    }
    
    public void pull() {
        left.set(ControlMode.PercentOutput, -.5);
        right.set(ControlMode.PercentOutput, .5);
        grip.set(Value.kReverse);
    }

    public void grab() {
        left.set(ControlMode.PercentOutput, -.5);
        right.set(ControlMode.PercentOutput, .5);
        grip.set(Value.kForward);
    }

    public void release() {
        left.neutralOutput();
        right.neutralOutput();
        grip.set(Value.kReverse);
    }

    public void toss() {
        left.set(ControlMode.PercentOutput, .75);
        right.set(ControlMode.PercentOutput, -.75);
        grip.set(Value.kForward);
    }

    public void hold() {
        grip.set(Value.kForward);
        left.neutralOutput();
        right.neutralOutput();
    }

    public void diff(double lMotor, double rMotor) {
        left.set(ControlMode.PercentOutput, lMotor);
        right.set(ControlMode.PercentOutput, rMotor);
    }

    public void tilt(double angle) {
        tilt.set(ControlMode.Position, angle * TICKS_PER_DEGREE);
    }

    public double getTilt() {
        return tilt.getSelectedSensorPosition(0) / TICKS_PER_DEGREE;
    }

    public boolean isLimitHit() {
        return tilt.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean isInitialized()
    {
        return isInitialized;
    }

    public double getTiltSpeed() {
        return (tilt.getSelectedSensorVelocity(0) * 10.0) / TICKS_PER_DEGREE;
    }
}
