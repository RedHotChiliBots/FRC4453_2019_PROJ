/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

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
  public XboxController operator = new XboxController(1);
  private Joystick btnBoard = new Joystick(2);

  // private JoystickButton climberClimb = new JoystickButton(driver,
  // RobotMap.Y_BUTTON);

  private JoystickButton climberDown = new JoystickButton(operator, RobotMap.A_BUTTON);
  private JoystickButton climberUp = new JoystickButton(operator, RobotMap.B_BUTTON);

  // Driver joystick controls
  // private JoystickButton switchToCargo = new JoystickButton(driver,
  // RobotMap.LEFT_BUMPER);
  // private JoystickButton switchToPanel = new JoystickButton(driver,
  // RobotMap.RIGHT_BUMPER);
  // private JoystickButton cargoRelease = new JoystickButton(driver,
  // RobotMap.RIGHT_BUMPER);
  // private JoystickButton cargoGrab = new JoystickButton(driver,
  // RobotMap.LEFT_BUMPER);

  private JoystickButton autoRetrieveScore = new JoystickButton(driver, RobotMap.A_BUTTON);

  private JoystickButton climberClimbNew = new JoystickButton(driver, RobotMap.Y_BUTTON);

  private JoystickButton liftReset = new JoystickButton(driver, RobotMap.X_BUTTON);

  // Operator joystick controls
  private JoystickButton switchToLevel1 = new JoystickButton(operator, RobotMap.X_BUTTON);
  private JoystickButton switchToLevel2 = new JoystickButton(operator, RobotMap.Y_BUTTON);
  // private JoystickButton switchToLevel3 = new JoystickButton(operator,
  // RobotMap.B_BUTTON);
  // private JoystickButton switchToShip = new JoystickButton(operator,
  // RobotMap.A_BUTTON);
  private JoystickButton switchToLoadingStation = new JoystickButton(operator, RobotMap.START);

  private JoystickButton liftGoToLevel = new JoystickButton(operator, RobotMap.BACK);

  private JoystickButton panelRelease = new JoystickButton(operator, RobotMap.RIGHT_BUMPER);
  private JoystickButton panelGrab = new JoystickButton(operator, RobotMap.LEFT_BUMPER);

  // ButtonBoard controls
  private JoystickButton bbSelectPanel = new JoystickButton(btnBoard, RobotMap.BTN_1);
  private JoystickButton bbSelectCargo = new JoystickButton(btnBoard, RobotMap.BTN_2);
  private JoystickButton bbSelectLoad = new JoystickButton(btnBoard, RobotMap.BTN_3);
  private JoystickButton bbSelectShip = new JoystickButton(btnBoard, RobotMap.BTN_4);
  private JoystickButton bbSelectLvl1 = new JoystickButton(btnBoard, RobotMap.BTN_5);
  private JoystickButton bbSelectLvl2 = new JoystickButton(btnBoard, RobotMap.BTN_6);
  private JoystickButton bbSelectLvl3 = new JoystickButton(btnBoard, RobotMap.BTN_7);
  private JoystickButton bbSelectLeft = new JoystickButton(btnBoard, RobotMap.BTN_8);
  private JoystickButton bbSelectCenter = new JoystickButton(btnBoard, RobotMap.BTN_9);
  private JoystickButton bbSelectRight = new JoystickButton(btnBoard, RobotMap.BTN_10);
  private JoystickButton bbGoToSelection = new JoystickButton(btnBoard, RobotMap.BTN_11);
  private JoystickButton bbAbort = new JoystickButton(btnBoard, RobotMap.BTN_12);

  /*
   * Andrew's new config private JoystickButton switchToCargo = new
   * JoystickButton(driver, RobotMap.RIGHT_BUMPER); private JoystickButton
   * switchToPanel = new JoystickButton(driver, RobotMap.LEFT_BUMPER);
   * 
   * private JoystickButton climberClimbNew = new JoystickButton(driver,
   * RobotMap.X_BUTTON);
   * 
   * private JoystickButton panelRelease = new JoystickButton(operator,
   * RobotMap.RIGHT_BUMPER); private JoystickButton panelGrip = new
   * JoystickButton(operator, RobotMap.LEFT_BUMPER);
   * 
   * private JoystickButton switchToLevel1 = new JoystickButton(operator,
   * RobotMap.Y_BUTTON); private JoystickButton switchToLevel2 = new
   * JoystickButton(operator, RobotMap.B_BUTTON); private JoystickButton
   * switchToLevel3 = new JoystickButton(operator, RobotMap.A_BUTTON); private
   * JoystickButton switchToShip = new JoystickButton(operator,
   * RobotMap.X_BUTTON);
   */
  private static final double DEADZONE = 0.2;

  public OI() {
  }

  public void init() {
    // switchToCargo.whenPressed(new SwitchToCargo());
    // switchToPanel.whenPressed(new SwitchToPanel());
    // cargoGrab.whenPressed(new CargoGrab());
    // cargoRelease.whenPressed(new CargoRelease());
    liftReset.whenPressed(new LiftStartup());
    panelGrab.whenPressed(new PanelGrab());
    panelRelease.whenPressed(new PanelRelease());
    // climberClimb.whileHeld(new ClimberClimb());
    climberClimbNew.whenPressed(new ClimberClimbNew());
    switchToLevel1.whenPressed(new SwitchToLevel1());
    switchToLevel2.whenPressed(new SwitchToLevel2());
    // switchToLevel3.whenPressed(new SwitchToLevel3());
    switchToLoadingStation.whenPressed(new SwitchToLoadingStation());
    // switchToShip.whenPressed(new SwitchToShip());
    climberDown.whenPressed(new ClimberExtend());
    climberUp.whenPressed(new ClimberRetract());
    autoRetrieveScore.whenPressed(new AutoRetrieveScore());
    liftGoToLevel.whenPressed(new LiftGoToLevel());

    bbSelectPanel.whenPressed(new SwitchToPanel());
    bbSelectCargo.whenPressed(new SwitchToCargo());
    bbSelectLoad.whenPressed(new SwitchToLoadingStation());
    bbSelectShip.whenPressed(new SwitchToShip());
    bbSelectLvl1.whenPressed(new SwitchToLevel1());
    bbSelectLvl2.whenPressed(new SwitchToLevel2());
    bbSelectLvl3.whenPressed(new SwitchToLevel3());
    bbSelectLeft.whenPressed(new SwitchToLeft());
    bbSelectCenter.whenPressed(new SwitchToCenter());
    bbSelectRight.whenPressed(new SwitchToRight());
    bbGoToSelection.whenPressed(new LiftGoToLevel());
    bbAbort.whenPressed(new Abort());

    /*
     * Andrew's new config switchToCargo.whenPressed(new SwitchToCargo());
     * switchToPanel.whenPressed(new SwitchToPanel());
     * climberClimbNew.whenPressed(new ClimberClimbNew()); panelGrip.whenPressed(new
     * PanelGrip()); panelRelease.whileHeld(new PanelRelease());
     * switchToLevel1.whenPressed(new SwitchToLevel1());
     * switchToLevel2.whenPressed(new SwitchToLevel2());
     * switchToLevel3.whenPressed(new SwitchToLevel3());
     * switchToShip.whenPressed(new SwitchToShip());
     */
  }

  public void setDriverRumble(GenericHID.RumbleType t) {
    driver.setRumble(t, 1);
  }

  public void resetDriverRumble(GenericHID.RumbleType t) {
    driver.setRumble(t, 0);
  }

  public void setOperatorRumble(GenericHID.RumbleType t) {
    operator.setRumble(t, 1);
  }

  public void resetOperatorRumble(GenericHID.RumbleType t) {
    operator.setRumble(t, 0);
  }

  public double getDriveX() {
    double v = driver.getX(Hand.kLeft);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }

  /*
   * Andrew's new config public double getDriveX() { double v =
   * driver.getX(Hand.kLeft); return Math.abs(v) < DEADZONE ? 0.0 : v; }
   */
  public double getDriveY() {
    double v = driver.getY(Hand.kLeft);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }

  /*
   * Andrew's new config public double getDriveY() { double v =
   * driver.getY(Hand.kLeft); return Math.abs(v) < DEADZONE ? 0.0 : v; }
   */
  public double getDriveR() {
    double v = driver.getX(Hand.kRight);
    return Math.abs(v) < DEADZONE ? 0.0 : v;
  }

  /*
   * Andrew's new config public double getDriveR() { double v =
   * driver.getX(Hand.kRight); return Math.abs(v) < DEADZONE ? 0.0 : v; }
   */
  public double getCargoL() {
    int v = operator.getPOV();
    if (v >= 0) {
      return calcCargo(v);
    } else {
      return 0.0;
    }
  }

  public double getCargoR() {
    int v = operator.getPOV();
    if (v >= 0) {
      return calcCargo(Math.abs(360 - v));
    } else {
      return 0.0;
    }
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
