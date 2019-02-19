/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Library;
import frc.robot.Gains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class UpperLift extends Subsystem {
  public WPI_TalonSRX motor1;
  public WPI_TalonSRX motor2;

  double _lockedDistance = 0;
  double _targetAngle = 0;

  private Gains kGains_Distance = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
  private Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.0);

  /**
   * Add your docs here.
   */
  public UpperLift() {
    motor1 = new WPI_TalonSRX(RobotMap.upperLiftMotor1);
    motor1.set(ControlMode.PercentOutput, 0.0);
    motor1.setSubsystem("UpperLift");

    motor2 = new WPI_TalonSRX(RobotMap.upperLiftMotor2);
    motor2.set(ControlMode.PercentOutput, 0.0);
    motor2.setSubsystem("UpperLift");

    Library.ConfigMotionMagic(motor1, motor2, kGains_Distance, kGains_Turning);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void lowerMotor(WPI_TalonSRX motor) {
    motor.set(ControlMode.PercentOutput, 0.2);
  }

  public void stopMotor(WPI_TalonSRX motor) {
    motor.stopMotor();
  }

  public void resetMotorConfig(double pos) {
    Library.ConfigMotionMagic(motor1, motor2, kGains_Distance, kGains_Turning);
  }

  // public void setPosMotor(WPI_TalonSRX motor, double pos) {
  public void setPosMotor(double pos) {
    Library.setSensorPosition(motor1, motor2, pos * Library.TICKS_PER_INCH);
    // motor.set(ControlMode.Position, (int) (pos * Library.TICKS_PER_INCH));
  }

  public void resetPosMotor(double pos) {
    Library.resetSensors(motor1, motor2, pos * Library.TICKS_PER_INCH);
    // motor.setSelectedSensorPosition((int) (pos * Library.TICKS_PER_INCH));
  }

  /*
   * public void setPos(double pos) { // setPosMotor(motor1, pos); //
   * setPosMotor(motor2, pos); setPosMotor(pos); }
   * 
   * public void resetPos(double pos) { // resetPosMotor(motor1, pos); //
   * resetPosMotor(motor2, pos); resetPosMotor(pos); }
   */
  /*
   * public void goToLevel(RobotMap.LEVEL level){ setPos((int) level); }
   */
}