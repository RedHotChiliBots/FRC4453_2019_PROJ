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
import frc.robot.commands.CargoTeleop;
import frc.robot.commands.ClimberClimb;
import frc.robot.commands.ClimberExtend;
import frc.robot.commands.ClimberRetract;
import frc.robot.commands.LiftStartup;
import frc.robot.commands.PanelGrip;
import frc.robot.commands.PanelRelease;
import frc.robot.commands.SwitchToCargo;
import frc.robot.commands.SwitchToLevel1;
import frc.robot.commands.SwitchToLevel2;
import frc.robot.commands.SwitchToLevel3;
import frc.robot.commands.SwitchToLoadingStation;
import frc.robot.commands.SwitchToPanel;
import frc.robot.commands.SwitchToShip;

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

  private XboxController driver = new XboxController(0);
  private XboxController operator = new XboxController(1);

  private JoystickButton switchToCargo = new JoystickButton(driver, RobotMap.A_BUTTON);
  private JoystickButton switchToPanel = new JoystickButton(driver, RobotMap.B_BUTTON);

  private JoystickButton liftReset = new JoystickButton(driver, RobotMap.X_BUTTON);

  private JoystickButton climberClimb = new JoystickButton(driver, RobotMap.Y_BUTTON);

  private JoystickButton panelRelease = new JoystickButton(driver, RobotMap.DPAD_X_AXIS);
  private JoystickButton panelGrip = new JoystickButton(driver, RobotMap.DPAD_Y_AXIS);

  private JoystickButton switchToLevel1 = new JoystickButton(driver, RobotMap.RIGHT_BUMPER);
  private JoystickButton switchToLevel2 = new JoystickButton(driver, RobotMap.LEFT_BUMPER);
  private JoystickButton switchToLevel3 = new JoystickButton(operator, RobotMap.RIGHT_BUMPER);
  private JoystickButton switchToLoadingStation = new JoystickButton(operator, RobotMap.LEFT_BUMPER);
  private JoystickButton switchToShip = new JoystickButton(driver, RobotMap.START);

  private JoystickButton climberDown = new JoystickButton(operator, RobotMap.A_BUTTON);
  private JoystickButton climberUp = new JoystickButton(operator, RobotMap.B_BUTTON);

  private static final double DEADZONE = 0.2;

  public OI() {

  }

  public void init() {
    switchToCargo.whenPressed(new SwitchToCargo());
    switchToPanel.whenPressed(new SwitchToPanel());
    liftReset.whenPressed(new LiftStartup());
    panelGrip.whenPressed(new PanelGrip());
    panelRelease.whenPressed(new PanelRelease());
    climberClimb.whenPressed(new ClimberClimb());
    switchToLevel1.whenPressed(new SwitchToLevel1());
    switchToLevel2.whenPressed(new SwitchToLevel2());
    switchToLevel3.whenPressed(new SwitchToLevel3());
    switchToLoadingStation.whenPressed(new SwitchToLoadingStation());
    switchToShip.whenPressed(new SwitchToShip());
    climberDown.whenPressed(new ClimberExtend());
    climberUp.whenPressed(new ClimberRetract());
  }

  public double getDriveX() {
    double v = driver.getX(Hand.kRight);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }

  public double getDriveY() {
    double v = driver.getY(Hand.kRight);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }

  public double getDriveR() {
    double v = driver.getX(Hand.kLeft);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }

  public double getCargoL() {
    int v = operator.getPOV();
    return calcCargo(v);
  }

  public double getCargoR() {
    int v = operator.getPOV();
    return calcCargo(Math.abs(360 - v));
  }

  private double calcCargo(int v) {
    double spd = 0.0;
    if ((v > 135) && (v < 225)) {
      spd = -1.0;
    } else if (v >= 225) {
      spd = 1.0;
    } else if (v > 90) {
      spd = 0.0;
    } else {
      spd = 1.0 - (v / 90.0);
    }
    return spd;
  }
}
