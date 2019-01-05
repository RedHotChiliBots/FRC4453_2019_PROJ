package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Shooter extends Subsystem {
    // private WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.SHOOTER_WIND_MOTOR);
    private DoubleSolenoid latch       = new DoubleSolenoid(RobotMap.SHOOTER_LATCH_SOLENOID, RobotMap.SHOOTER_RELEASE_SOLENOID);
    private DigitalInput   limitswitch = new DigitalInput(RobotMap.SHOOTER_LIMIT_SWITCH);
    // private Relay clutch = new Relay(RobotMap.SHOOTER_CLUTCH_RELAY);

    public class PrepareCommand extends CommandGroup {
        public PrepareCommand() {
            requires(Shooter.this);
            // addSequential(new PrepareCommand1());
            // addSequential(new PrepareCommand2());
            addSequential(new Command() { // Wait for Interrupt
                @Override
                protected boolean isFinished() {
                    return false;
                };
            });
        }
    }

    // private class PrepareCommand1 extends Command {

    //     public PrepareCommand1() {
    //         requires(Shooter.this);
    //         setInterruptible(false);
    //     }

    //     @Override
    //     protected void initialize() {
    //         System.out.println("Winding shooter...");
    //         latch.set(DoubleSolenoid.Value.kReverse);
    //         clutch.set(Relay.Value.kOn);
    //         motor.set(ControlMode.PercentOutput, 1);
    //     }
        
    //     protected void execute()
    //     {
    //         motor.set(ControlMode.PercentOutput, 1);
    //     }

    //     @Override
    //     protected boolean isFinished() {
    //         return limitswitch.get();
    //     }

    //     @Override
    //     protected void end() {
    //         latch.set(DoubleSolenoid.Value.kForward);
    //         motor.neutralOutput();
    //         System.out.println("Shooter Latched!");
    //     }
        
    //     protected void interrupted() {
    //         System.out.println("Winding interrupted!");
    //     }

    // }

    // private class PrepareCommand2 extends TimedCommand {

    //     public PrepareCommand2() {
    //         super(0.5);
    //         requires(Shooter.this);
    //         setInterruptible(false);
    //     }

    //     @Override
    //     protected void initialize() {

    //     }

    //     @Override
    //     protected void end() {
    //         clutch.set(Relay.Value.kOff);
    //         System.out.println("Clutch released!!");
    //     }

    // }

    public void fire() {
        latch.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new PrepareCommand());
    }
    
    public boolean isLimitHit() {
        return limitswitch.get();
    }
}
