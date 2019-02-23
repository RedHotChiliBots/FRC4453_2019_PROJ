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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class LowerLift extends Subsystem {
  public WPI_TalonSRX motor1;
  public WPI_TalonSRX motor2;

  /**
   * Add your docs here.
   */
  public LowerLift() {
    motor1 = new WPI_TalonSRX(RobotMap.lowerLiftMotor1);
    motor1.set(ControlMode.PercentOutput, 0.0);
    motor1.setSubsystem("LowerLift");

    /*
     * motor2 = new WPI_TalonSRX(RobotMap.lowerLiftMotor2);
     * motor2.set(ControlMode.PercentOutput, 0.0); motor2.setSubsystem("LowerLift");
     * 
     * Library.ConfigMotionMagic(motor1, motor2, kGains_Distance, kGains_Turning);
     */ }motor2=new WPI_TalonSRX(RobotMap.lowerLiftMotor2);motor2.set(ControlMode.PercentOutput,0.0);motor2.setSubsystem("LowerLift");

  Library.ConfigMotionMagic(motor1,motor2);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void lowerMotor() {
    motor1.set(ControlMode.PercentOutput, 0.2);
  }

  public void stopMotor() {
    motor1.stopMotor();
  }

  public void resetMotorConfig() {
    Library.ConfigMotionMagic(motor1, motor2);
  }

  // public void setPosMotor(WPI_TalonSRX motor, double pos) {
  public void setPosMotor(double pos) {
    // Library.setSensorPosition(motor1, pos * Library.TICKS_PER_INCH);
    // motor.set(ControlMode.Position, (int) (pos * Library.TICKS_PER_INCH));
  }

  public void resetPosMotor(double pos) {
    // Library.resetSensors(motor1, motor2, pos * Library.TICKS_PER_INCH);
    // motor.setSelectedSensorPosition((int) (pos * Library.TICKS_PER_INCH));
  }

  /*
   * public void goToLevel(RobotMap.LEVEL level){ setPos((int) level); }
   */
}
