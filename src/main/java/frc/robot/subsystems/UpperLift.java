/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class UpperLift extends PIDSubsystem {
  public WPI_TalonSRX motor1;
  public WPI_TalonSRX motor2;

  private static final int COUNTS_PER_REV_MOTOR = 12;
  private static final int GEAR_RATIO = 20;
  private static final int COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEAR_RATIO;
  private static final double TICKS_PER_INCH = COUNTS_PER_REV_GEARBOX; // Lead screw 1 in/rev

  /*
   * public static enum Level{ LEVEL1, LEVEL2, LEVEL3, LOADINGSTATION, SHIP }
   * 
   * public Level level = null;
   */
  /**
   * Add your docs here.
   */
  public UpperLift() {
    // Intert a subsystem name and PID values here
    super("SubsystemName", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    motor1 = new WPI_TalonSRX(RobotMap.upperLiftMotor1);
    motor1.configFactoryDefault();
    motor1.setNeutralMode(NeutralMode.Brake);
    motor1.set(0.0);
    motor1.setSubsystem("UpperLift");
    motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

    motor2 = new WPI_TalonSRX(RobotMap.upperLiftMotor2);
    motor2.configFactoryDefault();
    motor2.setNeutralMode(NeutralMode.Brake);
    motor2.set(0.0);
    motor2.setSubsystem("UpperLift");
    motor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

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
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setPosMotor(WPI_TalonSRX motor, double pos) {
    motor.set((int) (pos * TICKS_PER_INCH));
  }

  public void resetPosMotor(WPI_TalonSRX motor, double pos) {
    motor.setSelectedSensorPosition((int) (pos * TICKS_PER_INCH));
  }

  public void setPos(int pos) {
    setPosMotor(motor1, pos);
    setPosMotor(motor2, pos);
  }

  public void resetPos(int pos) {
    resetPosMotor(motor1, pos);
    resetPosMotor(motor2, pos);
  }
}