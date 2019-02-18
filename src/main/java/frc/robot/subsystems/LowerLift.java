/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class LowerLift extends PIDSubsystem {
  public WPI_TalonSRX motor1;
  public WPI_TalonSRX motor2;

  private static final int COUNTS_PER_REV_MOTOR = 12;
  private static final int GEAR_RATIO = 20;
  private static final int COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEAR_RATIO;
  private static final double TICKS_PER_INCH = COUNTS_PER_REV_GEARBOX; // Lead screw 1 in/rev

  /**
   * Add your docs here.
   */
  public LowerLift() {
    // Intert a subsystem name and PID values here
    super("SubsystemName", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    motor1 = new WPI_TalonSRX(RobotMap.lowerLiftMotor1);
    motor1.configFactoryDefault();
    motor1.setNeutralMode(NeutralMode.Brake);
    motor1.set(ControlMode.PercentOutput, 0.0);
    motor1.setSubsystem("LowerLift");
    motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
    motor1.setInverted(false);
    motor1.setSensorPhase(true);

    motor2 = new WPI_TalonSRX(RobotMap.lowerLiftMotor2);
    motor2.configFactoryDefault();
    motor2.setNeutralMode(NeutralMode.Brake);
    motor2.set(ControlMode.PercentOutput, 0.0);
    motor2.setSubsystem("LowerLift");
    motor2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
    motor2.setInverted(false);
    motor2.setSensorPhase(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
  }

  public void lowerMotor(WPI_TalonSRX motor) {
    motor.set(ControlMode.PercentOutput, 0.25);
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
