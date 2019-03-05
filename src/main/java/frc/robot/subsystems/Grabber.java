/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ACTION;
import frc.robot.RobotMap.DIR;
import frc.robot.RobotMap.MODE;
import frc.robot.commands.CargoTeleop;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {

  // Define Cargo Motors
  private WPI_TalonSRX motorL = null;
  private WPI_TalonSRX motorR = null;

  // Define Panel Solenoid
  private DoubleSolenoid panel = null;

  // Define Mode variable
  public MODE mode = null;
  public ACTION action = null;

  private DIR dir = null;

  public Grabber() {
    super("Grabber");

    // Initialize Cargo motors
    motorL = new WPI_TalonSRX(RobotMap.cargoGrabberMotorL);
    motorL.configFactoryDefault();
    motorL.setNeutralMode(NeutralMode.Brake);
    motorL.setSubsystem("Grabber");
    motorL.setInverted(true);
    motorL.set(ControlMode.PercentOutput, 0.0);
    motorL.configPeakOutputForward(+1.0, RobotMap.kTimeoutMs);
    motorL.configPeakOutputReverse(-1.0, RobotMap.kTimeoutMs);

    motorR = new WPI_TalonSRX(RobotMap.cargoGrabberMotorR);
    motorR.configFactoryDefault();
    motorR.setNeutralMode(NeutralMode.Brake);
    motorR.setSubsystem("Grabber");
    motorR.setInverted(false);
    motorR.set(ControlMode.PercentOutput, 0.0);
    motorR.configPeakOutputForward(+1.0, RobotMap.kTimeoutMs);
    motorR.configPeakOutputReverse(-1.0, RobotMap.kTimeoutMs);

    // Initialize Panel solenoid
    panel = new DoubleSolenoid(RobotMap.PanelGrabberReleaseSolenoid, RobotMap.PanelGrabberGripSolenoid);

    // Initialze Mode to PANEL
    mode = MODE.PANEL;
    action = ACTION.GRAB;

    dir = DIR.CENTER;

    // Add Sendable data to dashboard
    SmartDashboard.putData("Cargo motorL", motorL);
    SmartDashboard.putData("Cargo motorR", motorR);
    SmartDashboard.putData("Panel Grabber", panel);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CargoTeleop());
  }

  public void setMode(MODE m) {
    mode = m;
  }

  public void setAction(ACTION a) {
    action = a;
  }

  public MODE getMode() {
    return mode;
  }

  public ACTION getAction() {
    return action;
  }

  public void setDir(DIR d) {
    dir = d;
  }

  public DIR getDir() {
    return dir;
  }

  public double getMotorLCurrent() {
    return motorL.getOutputCurrent();
  }

  public double getMotorRCurrent() {
    return motorR.getOutputCurrent();
  }

  public void cargoGrab() {
    double spd = Robot.prefs.getDouble("CargoGrabSpd", 0.5);
    motorL.set(ControlMode.PercentOutput, spd);
    motorR.set(ControlMode.PercentOutput, spd);
  }

  public void cargoTeleop(double l, double r) {
    // System.out.println("POV Angle: " + Robot.oi.operator.getPOV());
    motorL.set(ControlMode.PercentOutput, -l);
    motorR.set(ControlMode.PercentOutput, -r);
  }

  public void cargoRel() {
    double spd = -Robot.prefs.getDouble("CargoRelSpd", 0.5);
    switch (dir) {
    case LEFT:
      motorL.set(ControlMode.PercentOutput, spd);
      break;
    case RIGHT:
      motorR.set(ControlMode.PercentOutput, spd);
      break;
    case CENTER:
      motorL.set(ControlMode.PercentOutput, spd);
      motorR.set(ControlMode.PercentOutput, spd);
      break;
    default:
    }
  }

  public void cargoStop() {
    motorL.stopMotor();
    motorR.stopMotor();
  }

  public void panelGrab() {
    panel.set(RobotMap.PanelGrab);
  }

  public void panelRel() {
    panel.set(RobotMap.PanelRelease);
  }

  /*****************************************************************
   * Panel/Cargo & Lift routines
   *****************************************************************/

  /*
   * public void switchPanelCargo(){ if (panelSelcted = true){ panelSelcted =
   * false; cargoSelected = true; }
   * 
   * if (cargoSelected = true){ cargoSelected = false; panelSelcted = true; } }
   * 
   * public boolean isPanelSelected(){ return panelSelcted; }
   * 
   * public boolean isCargoSelected(){ return cargoSelected; }
   */

  public void grabCargoPanel() {
    switch (mode) {
    case PANEL:
      panelGrab();
      break;
    case CARGO:
      cargoGrab();
      break;
    default:
    }
  }

  public void relCargoPanel() {
    switch (mode) {
    case PANEL:
      panelRel();
      break;
    case CARGO:
      cargoRel();
      break;
    default:
    }
  }

  /*
   * public void setDirection(CargoMotor dir) { Direction = dir; }
   */
}
