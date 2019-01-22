/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveTeleop;

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

	private AnalogInput	hiPressureSensor;
	private AnalogInput	loPressureSensor;

	private boolean collisionDetected = false;

	double last_world_linear_accel_x;
	double last_world_linear_accel_y;

	final static double kCollisionThreshold_DeltaG = 0.5f;

	// Define navX board
	public AHRS ahrs = null;

	private static final double CHASSIS_GEAR_RATIO = 1.0; // Encoder revs per wheel revs. TODO
  private static final double CHASSIS_ENCODER_TICKS_PER_REVOLUTION = 4096;//TODO
  private static final double CHASSIS_WHEEL_DIAMETER = 8.0; // inches
  private static final double CHASSIS_TICKS_PER_INCH = (CHASSIS_GEAR_RATIO * CHASSIS_ENCODER_TICKS_PER_REVOLUTION) / (CHASSIS_WHEEL_DIAMETER * Math.PI);

	private AnalogInput		    leftDistanceSensor		 = new AnalogInput(RobotMap.leftDistanceSensor);
	private AnalogInput		    rightDistanceSensor		 = new AnalogInput(RobotMap.rightDistanceSensor);


	public Chassis() {
		frontleft = new WPI_TalonSRX(RobotMap.frontLeftMotor);
		frontright = new WPI_TalonSRX(RobotMap.frontRightMotor);
		backleft = new WPI_TalonSRX(RobotMap.backLeftMotor);
		backright = new WPI_TalonSRX(RobotMap.backRightMotor);

		frontleft.set(0.0);
    frontleft.setSubsystem("Chassis");
		frontleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
		frontright.set(0.0);
    frontright.setSubsystem("Csassis");
	  frontright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
		backleft.set(0.0);
    backleft.setSubsystem("Chassis");
	  backleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
		backright.set(0.0);
    backright.setSubsystem("Chassis");
	  backright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
 
		drive = new MecanumDrive(frontleft, frontright, backleft, backright);

		hiPressureSensor = new AnalogInput(RobotMap.highPressureSensor);
		loPressureSensor = new AnalogInput(RobotMap.lowPressureSensor);
		
		try {
			/* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
			/* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
			ahrs = new AHRS(SPI.Port.kMXP); 
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
	}
	
	public void driveChassis(double x, double y, double r) {
		drive.driveCartesian(y, x, r);
	}

	public void driveTeleop(){
		double x = Robot.oi.getDriveX();
    double y = Robot.oi.getDriveY();
		double r = Robot.oi.getDriveR();
		driveChassis(x,y,r);
	}

	public void driveVision(){

	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
	// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveTeleop());
	}
	
	public void followLine(){

	}

	public void distFromBay(){

	}

	public void findJerk(){
		double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_Y = ahrs.getWorldLinearAccelY();
		double currentJerkY = curr_world_linear_accel_Y - last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_Y;

		if ( (Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)||
			(Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ){
			collisionDetected = true;
		}
	}

	public boolean IsCollisionDetected(){
		return collisionDetected;
	}

	public void resetCollisionDetected(){
		collisionDetected = false;
	}
}
