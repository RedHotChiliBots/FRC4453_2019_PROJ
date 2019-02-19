/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Gains;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class LowerLift extends Subsystem {
  public WPI_TalonSRX motor1;
  public WPI_TalonSRX motor2;

  private static final int COUNTS_PER_REV_MOTOR = 12;
  private static final int GEAR_RATIO = 20;
  private static final int COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEAR_RATIO;
  private static final double TICKS_PER_INCH = COUNTS_PER_REV_GEARBOX; // Lead screw 1 in/rev

  double _lockedDistance = 0;
  double _targetAngle = 0;

  private Gains kGains_Distance = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
  private Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.0);

  /**
   * Add your docs here.
   */
  public LowerLift() {
    motor1 = new WPI_TalonSRX(RobotMap.lowerLiftMotor1);
    motor1.set(ControlMode.PercentOutput, 0.0);
    motor1.setSubsystem("LowerLift");

    motor2 = new WPI_TalonSRX(RobotMap.lowerLiftMotor2);
    motor2.set(ControlMode.PercentOutput, 0.0);
    motor2.setSubsystem("LowerLift");

    RobotMap.ConfigMotionMagic(motor1, motor2, kGains_Distance, kGains_Turning);
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
    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor1.set(ControlMode.Position, pos);
  }

  public void setPosMotor(WPI_TalonSRX motor, double pos) {
    motor.set(ControlMode.Position, (int) (pos * TICKS_PER_INCH));
  }

  public void resetPosMotor(WPI_TalonSRX motor, double pos) {
    motor.setSelectedSensorPosition((int) (pos * TICKS_PER_INCH));
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
