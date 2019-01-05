/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4453.robot;

import org.usfirst.frc.team4453.robot.commands.*;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private Joystick	   drive    = new Joystick(RobotMap.FIRST_CONTROLLER);
    private XboxController operator = new XboxController(RobotMap.SECOND_CONTROLLER);

    private JoystickButton quickTurn1 = new JoystickButton(drive, RobotMap.BUTTON_5);
    private JoystickButton quickTurn2 = new JoystickButton(drive, RobotMap.BUTTON_4);
    private JoystickButton shiftHigh  = new JoystickButton(drive, RobotMap.BUTTON_3);
    private JoystickButton shiftLow   = new JoystickButton(drive, RobotMap.BUTTON_2);

    //private JoystickButton shooterShoot = new JoystickButton(operator, RobotMap.Y_BUTTON);

    private JoystickButton grabberGrab    = new JoystickButton(operator, RobotMap.A_BUTTON);
    private JoystickButton grabberRelease = new JoystickButton(operator, RobotMap.B_BUTTON);
    private JoystickButton grabberThrow   = new JoystickButton(operator, RobotMap.X_BUTTON);
    private JoystickButton grabberPull    = new JoystickButton(operator, RobotMap.Y_BUTTON);

    private JoystickButton wingsLift = new JoystickButton(operator, RobotMap.LEFT_BUMPER);
    private JoystickButton wingsDrop = new JoystickButton(operator, RobotMap.RIGHT_BUMPER);

    private JoystickButton climberClimb = new JoystickButton(operator, RobotMap.START);
    
    private JoystickButton climberControl = new JoystickButton(operator, RobotMap.BACK);
    
    private Trigger hookRaise = new Trigger() {
                    @Override
                    public boolean get() {
                        return Math.abs(operator.getTriggerAxis(Hand.kLeft)) > 0.1;
                    }
                    };

   private Trigger hookLower = new Trigger() {
                    @Override
                    public boolean get() {
                        return Math.abs(operator.getTriggerAxis(Hand.kRight)) > 0.1;
                    }
                    };

    public OI() {
        shiftHigh.whenPressed(new ChassisShiftHigh());
        shiftLow.whenPressed(new ChassisShiftLow());

        grabberGrab.whileHeld(new GrabberGrab());
        grabberGrab.whenReleased(new GrabberHold());
        grabberRelease.whileHeld(new GrabberRelease());
        grabberThrow.whileHeld(new GrabberThrow());
        grabberThrow.whenReleased(new GrabberHold());
        grabberPull.whileHeld(new GrabberPull());
        grabberPull.whenReleased(new GrabberHold());
        
        wingsLift.whenPressed(new WingsLift());
        wingsDrop.whenPressed(new WingsDrop());

        //hookRaise.whileActive(new HookTeleop());
        hookRaise.whileActive(new HookRaise());
        //hookLower.whenInactive(new HookLower());
        hookLower.whileActive(new HookLowerFast());
        //shooterShoot.whenPressed(new ShooterFire());
        
        climberClimb.whileHeld(new ClimberClimb());
        
        climberControl.whileActive(new ClimberControl());;
        
    }

    public double getSpdAxis() {
        return -drive.getY();
    }

    public double getTurnAxis() {
        return -drive.getX();
    }

    public double getTiltAxis() {
        return operator.getY(Hand.kRight);
    }

    public double getGrabDiffAxis() {
        return operator.getX(Hand.kLeft);
    }

    public double getGrabSpeedAxis() {
        return operator.getY(Hand.kLeft);
    }
    
    public double getClimbControl()
    {
        return operator.getX(Hand.kRight);
    }

    public boolean isQuickTurn() {
        return quickTurn1.get() || quickTurn2.get();
    }

    public boolean isWingsLift() {
        return operator.getBumper(Hand.kLeft);
    }

    public boolean isWingsDrop() {
        return operator.getBumper(Hand.kRight);
    }

    public double getHookSpeed() {
        return operator.getTriggerAxis(Hand.kLeft) - operator.getTriggerAxis(Hand.kRight);
    }
    
    public int getDPad() {
        int p = operator.getPOV();
        if(p < 0)
        {
            p+=360;
        }
        return p;
    }
    
    public boolean climbControlLeft() {
        return drive.getRawButton(8);
    }
    
    public boolean climbControlRight() {
        return drive.getRawButton(9);
    }
};
