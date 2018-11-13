package org.usfirst.frc.team4453.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team4453.robot.RobotMap;
import org.usfirst.frc.team4453.robot.commands.HookLower;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Hook extends Subsystem {

    // PID Constants
    // private final int slotIdx = 0;
    // private final int pidLoopIdx  = 0;
    private final int timeOutMs   = 10;
    // private final boolean sensorPhase = true;
    private final boolean motorInvert = true;
    //private final double	 kF	     = 0.0;
    //private final double	 kP	     = 2.0;
    //private final double	 kI	     = 0.0;
    //private final double	 kD	     = 0.0;

    // Encoder Resolution
    //
    //private final int	COUNTS_PER_REV_MOTOR   = 12;	// 12 cpr quadrature - RS7 Encoder from Armabot
    //private final int	GEARBOX_RATIO	       = 20;
    //private final double	SHAFT_DIAMETER	       = 1.0;
    //private final int	COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEARBOX_RATIO;
    //private final double	INCHES_PER_REV	       = SHAFT_DIAMETER * Math.PI;
   // private final double	COUNTS_PER_INCH	       = COUNTS_PER_REV_GEARBOX / INCHES_PER_REV;

    //private final static double MAX_HEIGHT = 6.5*12;
    
    // Motor
    private final WPI_TalonSRX hookLift = new WPI_TalonSRX(RobotMap.CLIMBER_HOOK_MOTOR);

    private class InitCommand extends Command {

        public InitCommand()
        {
            requires(Hook.this);
            setInterruptible(false);
        }
        
        protected void initialize() {
            //hookLift.set(ControlMode.PercentOutput, -.3);
        }
        
        @Override
        protected boolean isFinished() {
            //return hookLift.getSensorCollection().isRevLimitSwitchClosed();
            return true;
        }
        
        protected void end() {
            //hookLift.neutralOutput();
            //hookLift.setSelectedSensorPosition(pidLoopIdx, 0, timeOutMs);
        }
        
    }
    
    public Hook() {
        // choose the sensor
        //hookLift.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidLoopIdx, timeOutMs);
        //hookLift.setSensorPhase(sensorPhase);
        hookLift.setInverted(motorInvert);

        // set the peak, nominal outputs
        hookLift.configNominalOutputForward(0, timeOutMs);
        hookLift.configNominalOutputReverse(0, timeOutMs);
        hookLift.configPeakOutputForward(.5, timeOutMs);
        hookLift.configPeakOutputReverse(-.5, timeOutMs);
        
        //hookLift.configClosedloopRamp(0, timeOutMs);

        // set allowable close-loop error
        //hookLift.configAllowableClosedloopError(pidLoopIdx, 0, timeOutMs);
        
        //hookLift.configForwardSoftLimitThreshold((int) (MAX_HEIGHT * COUNTS_PER_INCH), timeOutMs);
        //hookLift.configReverseSoftLimitThreshold(0, timeOutMs);
        //hookLift.configForwardSoftLimitEnable(true, timeOutMs);
        //hookLift.configReverseSoftLimitEnable(true, timeOutMs);
        
        
        // set closed loop gains in slot0
        //hookLift.config_kF(pidLoopIdx, kF, timeOutMs);
        //hookLift.config_kP(pidLoopIdx, kP, timeOutMs);
        //hookLift.config_kI(pidLoopIdx, kI, timeOutMs);
        //hookLift.config_kD(pidLoopIdx, kD, timeOutMs);
    }

    //public double getDistance() {
//	return hookLift.getSensorCollection().getQuadraturePosition() / COUNTS_PER_INCH;
    //}

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new HookLower());
    }
    
    public void init() {
        new InitCommand().start();
    }

    //public void set(double setPoint) {
//	hookLift.set(ControlMode.Position, setPoint * COUNTS_PER_INCH);
   // }
    
    public void raise() {
        hookLift.set(ControlMode.PercentOutput, .1); //TODO
    }
    
    public void lower() {
        hookLift.set(ControlMode.PercentOutput, -.1); //TODO
    }
    
    public void lowerFast() {
        hookLift.set(ControlMode.PercentOutput, -.3); //TODO
    }

    public void stop() {
        hookLift.neutralOutput();
    }

    //public void resetEncoder() {
//	hookLift.getSensorCollection().setQuadraturePosition(0, 100);
   // }
    
    public boolean isLimitHit() {
        return hookLift.getSensorCollection().isRevLimitSwitchClosed();
    }
}
