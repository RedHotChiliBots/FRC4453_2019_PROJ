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

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.CargoStop;
import frc.robot.commands.CargoTeleop;

/**
 * Add your docs here.
 */
public class CargoGrabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX motor1;
  private WPI_TalonSRX motor2;

  public static enum CargoMotor {
    LEFT, RIGHT, CENTER
  }

  // public CargoMotor Direction = CargoMotor.CENTER;

  public CargoGrabber() {
    motor1 = new WPI_TalonSRX(RobotMap.cargoGrabberMotor1);
    motor1.configFactoryDefault();
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2 = new WPI_TalonSRX(RobotMap.cargoGrabberMotor2);
    motor2.configFactoryDefault();
    motor2.setNeutralMode(NeutralMode.Brake);
    motor2.setInverted(true);

    SmartDashboard.putData(motor1);
    SmartDashboard.putData(motor2);
  }

  @Override
  public void initDefaultCommand() { // stop
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new CargoTeleop());
    // setDefaultCommand(new CargoStop());
  }

  public void setGrabRel(double l, double r) {
    motor1.set(ControlMode.PercentOutput, l);
    motor2.set(ControlMode.PercentOutput, r);
  }

  public void grab() {
    double spd = Robot.prefs.getDouble("CargoGrabSpd", 0.5);
    motor1.set(ControlMode.PercentOutput, -spd);
    motor2.set(ControlMode.PercentOutput, spd);
  }

  public void release(CargoMotor dir) {
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

  public void stop() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  /*
   * public void setDirection(CargoMotor dir) { Direction = dir; }
   */

}
