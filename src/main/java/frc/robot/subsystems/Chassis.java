/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import frc.robot.RobotMap.LEVEL;
import frc.robot.commands.ChassisDriveTeleop;
import frc.robot.subsystems.CargoGrabber.CargoMotor;

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public WPI_TalonSRX frontleft;
	public WPI_TalonSRX frontright;
	public WPI_TalonSRX backleft;
	public WPI_TalonSRX backright;
	private MecanumDrive drive;

	private AnalogInput hiPressureSensor;
	private AnalogInput loPressureSensor;

	private boolean collisionDetected = false;

	// private boolean panelSelcted = true;
	// private boolean cargoSelected = false;

	public enum Mode {
		PANEL, CARGO
	}

	public Mode mode = Mode.PANEL;

	public LEVEL level = LEVEL.LEVEL1;

	double last_world_linear_accel_x;
	double last_world_linear_accel_y;

	final static double kCollisionThreshold_DeltaG = 0.2f;

	// Define navX board
	public AHRS ahrs = null;

	private static final double CHASSIS_GEAR_RATIO = 1.0; // Encoder revs per wheel revs.
	private static final double CHASSIS_ENCODER_TICKS_PER_REVOLUTION = 4096; // Quad encoder, counts per rev
	private static final double CHASSIS_WHEEL_DIAMETER = 8.0; // inches
	private static final double CHASSIS_TICKS_PER_INCH = (CHASSIS_GEAR_RATIO * CHASSIS_ENCODER_TICKS_PER_REVOLUTION)
			/ (CHASSIS_WHEEL_DIAMETER * Math.PI);

	private static final double PRESSURE_SENSOR_INPUTVOLTAGE = 5.0;

	public Chassis() {
		frontleft = new WPI_TalonSRX(RobotMap.frontLeftMotor);
		frontright = new WPI_TalonSRX(RobotMap.frontRightMotor);
		backleft = new WPI_TalonSRX(RobotMap.backLeftMotor);
		backright = new WPI_TalonSRX(RobotMap.backRightMotor);

		frontleft.configFactoryDefault();
		frontleft.set(ControlMode.PercentOutput, 0.0);
		frontleft.setNeutralMode(NeutralMode.Brake);
		frontleft.setSubsystem("Chassis");
		frontleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

		frontright.configFactoryDefault();
		frontright.set(ControlMode.PercentOutput, 0.0);
		frontright.setNeutralMode(NeutralMode.Brake);
		frontright.setSubsystem("Csassis");
		frontright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

		backleft.configFactoryDefault();
		backleft.set(ControlMode.PercentOutput, 0.0);
		backleft.setNeutralMode(NeutralMode.Brake);
		backleft.setSubsystem("Chassis");
		backleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

		backright.configFactoryDefault();
		backright.set(ControlMode.PercentOutput, 0.0);
		backright.setNeutralMode(NeutralMode.Brake);
		backright.setSubsystem("Chassis");
		backright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

		drive = new MecanumDrive(frontleft, frontright, backleft, backright);

		hiPressureSensor = new AnalogInput(RobotMap.highPressureSensor);
		loPressureSensor = new AnalogInput(RobotMap.lowPressureSensor);

		try {
			/* Communicate w/navX-MXP via the MXP SPI Bus. */
			/* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
			/*
			 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
			 * details.
			 */
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}

		SmartDashboard.putData(frontleft);
		SmartDashboard.putData(frontright);
		SmartDashboard.putData(backleft);
		SmartDashboard.putData(backright);
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new ChassisDriveTeleop());
	}

	public void driveChassis(double x, double y, double r) {
		drive.driveCartesian(y, x, r, ahrs.getYaw());
	}

	public void driveTeleop() {
		double x = 0.0;
		double y = 0.0;
		double r = 0.0;

		switch (mode) {
		case CARGO:
			x = Robot.oi.getDriveX();
			y = Robot.oi.getDriveY();
			break;

		case PANEL:
			x = -Robot.oi.getDriveX();
			y = -Robot.oi.getDriveY();
			break;

		default:
		}

		r = Robot.oi.getDriveR();
		driveChassis(x, y, r);
	}

	public void driveVision() {
	}

	public void followLine() {
	}

	public void distFromBay() {
	}

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

	public void setMode(Mode m) {
		mode = m;
	}

	public void findJerk() {
		double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_Y = ahrs.getWorldLinearAccelY();
		double currentJerkY = curr_world_linear_accel_Y - last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_Y;

		/*
		 * if (panelSelcted = true){ if (currentJerkX > kCollisionThreshold_DeltaG &&
		 * Math.abs(currentJerkY) < 0.1){ collisionDetected = true; } }
		 * 
		 * if (cargoSelected = true){ if (currentJerkX < -kCollisionThreshold_DeltaG &&
		 * Math.abs(currentJerkY) < 0.1){ collisionDetected = true; } }
		 */
		if (mode == Mode.PANEL) {
			if (currentJerkX > kCollisionThreshold_DeltaG && Math.abs(currentJerkY) < 0.1) {
				collisionDetected = true;
			}
		}

		if (mode == Mode.CARGO) {
			if (currentJerkX < -kCollisionThreshold_DeltaG && Math.abs(currentJerkY) < 0.1) {
				collisionDetected = true;
			}
		}

	}

	public boolean IsCollisionDetected() {
		return collisionDetected;
	}

	public void resetCollisionDetected() {
		collisionDetected = false;
	}

	public double getPitch() {
		return ahrs.getRoll();
	}

	public double getRoll() {
		return ahrs.getPitch();
	}

	public double getYaw() {
		return ahrs.getYaw();
	}

	public double getLoPressure() {
		return 250.0 * (loPressureSensor.getVoltage() / PRESSURE_SENSOR_INPUTVOLTAGE) - 25.0; // ToDo
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / PRESSURE_SENSOR_INPUTVOLTAGE) - 25.0;
	}

	public void cargoPanelGrab(Mode mode) {
		if (mode == Mode.PANEL) {
			Robot.panel.grab();
		} else {
			Robot.cargo.grab();
		}
	}

	public void cargoPanelRelease(Mode mode, double l, double r) {
		if (mode == Mode.PANEL) {
			Robot.panel.release();
		} else {
			Robot.cargo.setGrabRel(l, r);
		}
	}

	public void setPos(double pos) {
		frontleft.set(ControlMode.Position, (int) (pos * CHASSIS_TICKS_PER_INCH));
	}

	public void reset() {
		frontleft.setSelectedSensorPosition((int) (0.0 * CHASSIS_TICKS_PER_INCH));
	}

	public void setPercentOut() {
		frontleft.set(ControlMode.PercentOutput, 0.0);
		frontright.set(ControlMode.PercentOutput, 0.0);
		backleft.set(ControlMode.PercentOutput, 0.0);
		backright.set(ControlMode.PercentOutput, 0.0);
	}

	private static final double mph = 3.0;
	private static final double inches = mph * 5280.0 * 12.0;
	private static final double mSec = 1000.0;
	private static final double cpr = 4096;
	private static final int vel = (int) (((inches / mSec) * 100) * cpr);

	public void setFollow() {
		reset();
		// TalonSRXPIDSetConfiguration pid =
		frontleft.set(ControlMode.Position, (int) (0.0 * CHASSIS_TICKS_PER_INCH));
		frontright.set(ControlMode.Follower, frontleft.getDeviceID());
		backleft.set(ControlMode.Follower, frontleft.getDeviceID());
		backright.set(ControlMode.Follower, frontleft.getDeviceID());
		frontleft.configMotionCruiseVelocity(vel);
		// frontleft.configMotionAcceleration(sensorUnitsPer100msPerSec);
		// frontleft.configurePID(pid);
	}

}
