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
import frc.robot.RobotMap.MODE;

import frc.robot.commands.GrabberStop;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {

  // Define Cargo Motors
  private WPI_TalonSRX motor1 = null;
  private WPI_TalonSRX motor2 = null;

  // Define Panel Solenoid
  private DoubleSolenoid panel = null;

  // Define Mode variable
  private MODE mode = null;

  // public CargoMotor Direction = CargoMotor.CENTER;

  public Grabber() {

    // Initialize Cargo motors
    motor1 = new WPI_TalonSRX(RobotMap.cargoGrabberMotor1);
    motor1.configFactoryDefault();
    motor1.setNeutralMode(NeutralMode.Brake);
    motor1.setSubsystem("Grabber");
    motor2.setInverted(false);
    motor1.set(ControlMode.PercentOutput, 0.0);

    motor2 = new WPI_TalonSRX(RobotMap.cargoGrabberMotor2);
    motor2.configFactoryDefault();
    motor2.setNeutralMode(NeutralMode.Brake);
    motor2.setSubsystem("Grabber");
    motor2.setInverted(true);
    motor2.set(ControlMode.PercentOutput, 0.0);

    // Initialize Panel solenoid
    panel = new DoubleSolenoid(RobotMap.PanelGrabberReleaseSolenoid, RobotMap.PanelGrabberGripSolenoid);

    // Initialze Mode to PANEL
    mode = MODE.PANEL;

    // Add Sendable data to dashboard
    SmartDashboard.putData("Cargo Motor1", motor1);
    SmartDashboard.putData("Cargo Motor2", motor2);
    SmartDashboard.putData("Panel Grabber", panel);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new GrabberStop());
  }

  public void setMode(MODE m) {
    mode = m;
  }

  public MODE getMode() {
    return mode;
  }

  public void cargoGrab() {
    double spd = Robot.prefs.getDouble("CargoGrabSpd", 0.5);
    motor1.set(ControlMode.PercentOutput, -spd);
    motor2.set(ControlMode.PercentOutput, spd);
  }

  public void cargoRel(double l, double r) {
    motor1.set(ControlMode.PercentOutput, l);
    motor2.set(ControlMode.PercentOutput, r);
  }

  public void cargoRel(RobotMap.CargoMotor dir) {
    double spd = Robot.prefs.getDouble("CargoRelSpd", 0.5);
    switch (dir) {
    case LEFT:
      motor1.set(ControlMode.PercentOutput, spd);
      break;
    case RIGHT:
      motor2.set(ControlMode.PercentOutput, -spd);
      break;
    case CENTER:
      motor1.set(ControlMode.PercentOutput, spd);
      motor2.set(ControlMode.PercentOutput, -spd);
      break;
    default:
    }
  }

  public void cargoStop() {
    motor1.stopMotor();
    motor2.stopMotor();
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

  public void grabCargoPanel(MODE mode) {
    if (mode == MODE.PANEL) {
      panelGrab();
    } else {
      cargoGrab();
    }
  }

  public void relCargoPanel(MODE mode, double l, double r) {
    if (mode == MODE.PANEL) {
      panelRel();
    } else {
      cargoRel(l, r);
    }
  }

  /*
   * public void setDirection(CargoMotor dir) { Direction = dir; }
   */
}
