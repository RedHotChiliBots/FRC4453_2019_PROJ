/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
	private WPI_TalonSRX frontleft;
	private WPI_TalonSRX frontright;
	private WPI_TalonSRX backleft;
	private WPI_TalonSRX backright;
	private MecanumDrive drive;

	public Chassis(){
		frontleft = new WPI_TalonSRX(RobotMap.frontLeftMotor);
		frontright = new WPI_TalonSRX(RobotMap.frontRightMotor);
		backleft = new WPI_TalonSRX(RobotMap.backLeftMotor);
		backright = new WPI_TalonSRX(RobotMap.backRightMotor);
		drive = new MecanumDrive(frontleft, frontright, backleft, backright);
	}
	
	public void drivechassis(double x, double y, double r){
		drive.driveCartesian(x, y, r);
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
	// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new Drive());
  }
}
