/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.LowerLiftResetMotor;
import frc.robot.commands.SwitchToCargo;
//import frc.robot.commands.SwitchToCargo;
//import frc.robot.commands.SwitchToPanel;
import frc.robot.commands.SwitchToPanel;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public XboxController driver   = new XboxController(0);
  public XboxController operator = new XboxController(1);

  private JoystickButton switchToCargo = new JoystickButton(driver, RobotMap.A_BUTTON);
  private JoystickButton switchToPanel = new JoystickButton(driver, RobotMap.B_BUTTON);
  private JoystickButton liftReset = new JoystickButton(driver, RobotMap.X_BUTTON);

  private static final double DEADZONE = 0.2;

  public OI(){

  }

  public void init(){
    switchToCargo.whenPressed(new SwitchToCargo());
    switchToPanel.whenPressed(new SwitchToPanel());
    liftReset.whenPressed(new LowerLiftResetMotor(Robot.lLift.motor1, Robot.prefs.getDouble("LLMotorReset", -1)));
  }

  public double getDriveX() {
    double v = driver.getX(Hand.kRight);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }

  public double getDriveY() {
    double v = -driver.getY(Hand.kRight);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }

  public double getDriveR() {
    double v = driver.getX(Hand.kLeft);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }


}
