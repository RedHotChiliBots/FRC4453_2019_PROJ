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
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

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

  private final double target_turn = 0.0; // Always moving straight line

  public void resetMotorConfig(double pos) {
    motor2.set(ControlMode.MotionMagic, pos, DemandType.AuxPID, target_turn);
    motor1.follow(motor2, FollowerType.AuxOutput1);
    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor1.set(ControlMode.Position, pos);
  }

  public void setPosMotor(WPI_TalonSRX motor, double pos) {
    motor.set(ControlMode.Position, (int) (pos * Library.TICKS_PER_INCH));
  }

  public void resetPosMotor(WPI_TalonSRX motor, double pos) {
    motor.setSelectedSensorPosition((int) (pos * Library.TICKS_PER_INCH));
  }

  public void setPos(double pos) {
    setPosMotor(motor1, pos);
    setPosMotor(motor2, pos);
  }

  public void resetPos(double pos) {
    resetPosMotor(motor1, pos);
    resetPosMotor(motor2, pos);
  }

  /*
   * public void goToLevel(RobotMap.LEVEL level){ setPos((int) level); }
   */
}