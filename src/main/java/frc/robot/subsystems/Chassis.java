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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.MODE;
import frc.robot.commands.ChassisDriveTeleop;

/**
 * Chassis - Manage Drive Train
 * 
 * The chassis subsystem is responsible for controlling the drive wheels in
 * Mecanum mode for TeleOp and Autonomous modes.
 * 
 * In TeleOp mode the driver's joystick controls the robot using field oriented
 * mode where, y pushes and pulls the robot away and towards the driver x moves
 * the robot left and right of the driver r rotates the robots orientation
 * 
 * In Autonomous mode the robot is controlled by the error terms provided by the
 * Vision code. The x error term provides the PID error causing the robot to
 * move left or right to center on the line. The r error term provides the PID
 * error causing the robot to rotate facing the scoring face. The y value is a
 * constant speed until "jerk" is detected when the robot stops.
 * 
 * "Jerk" is declared when deceleration of the robot is detected in its forward
 * direction. This condition is used to identify the scoring face and stop,
 * score.
 */
public class Chassis extends Subsystem {

	// Wheels and Drive type
	private WPI_TalonSRX frontleft = null;
	private WPI_TalonSRX frontright = null;
	private WPI_TalonSRX backleft = null;
	private WPI_TalonSRX backright = null;
	private MecanumDrive drive = null;

	// "jerk" detection
	private boolean collisionDetected = false;

	private double last_world_linear_accel_x = 0.0;
	private double last_world_linear_accel_y = 0.0;

	private final double kCollisionThreshold_DeltaG = 0.2;

	// Define navX board
	public AHRS ahrs = null;

	// Hi & Lo Pressure Sensors
	private AnalogInput hiPressureSensor = null;
	private AnalogInput loPressureSensor = null;
	private static final double PRESSURE_SENSOR_INPUTVOLTAGE = 5.0;

	// Define Encoder constants
	private static final double CHASSIS_GEAR_RATIO = 1.0; // Encoder revs per wheel revs.
	private static final double CHASSIS_ENCODER_TICKS_PER_REVOLUTION = 4096; // Quad encoder, counts per rev
	private static final double CHASSIS_WHEEL_DIAMETER = 8.0; // inches
	private static final double CHASSIS_TICKS_PER_INCH = (CHASSIS_GEAR_RATIO * CHASSIS_ENCODER_TICKS_PER_REVOLUTION)
			/ (CHASSIS_WHEEL_DIAMETER * Math.PI);

	public Chassis() {

		// Initialize drive train for Mechanam
		frontleft = new WPI_TalonSRX(RobotMap.frontLeftMotor);
		frontright = new WPI_TalonSRX(RobotMap.frontRightMotor);
		backleft = new WPI_TalonSRX(RobotMap.backLeftMotor);
		backright = new WPI_TalonSRX(RobotMap.backRightMotor);

		frontleft.configFactoryDefault();
		frontleft.set(ControlMode.PercentOutput, 0.0);
		frontleft.setNeutralMode(NeutralMode.Brake);
		frontleft.setSubsystem("Chassis");
		frontleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, // Local Feedback Source
				RobotMap.PID_PRIMARY, // PID Slot for Source [0, 1]
				RobotMap.kTimeoutMs); // Configuration Timeout
		frontright.configFactoryDefault();
		frontright.set(ControlMode.PercentOutput, 0.0);
		frontright.setNeutralMode(NeutralMode.Brake);
		frontright.setSubsystem("Csassis");
		frontright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, // Local Feedback Source
				RobotMap.PID_PRIMARY, // PID Slot for Source [0, 1]
				RobotMap.kTimeoutMs); // Configuration Timeout
		backleft.configFactoryDefault();
		backleft.set(ControlMode.PercentOutput, 0.0);
		backleft.setNeutralMode(NeutralMode.Brake);
		backleft.setSubsystem("Chassis");
		backleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, // Local Feedback Source
				RobotMap.PID_PRIMARY, // PID Slot for Source [0, 1]
				RobotMap.kTimeoutMs); // Configuration Timeout, 0, 100);
		backright.configFactoryDefault();
		backright.set(ControlMode.PercentOutput, 0.0);
		backright.setNeutralMode(NeutralMode.Brake);
		backright.setSubsystem("Chassis");
		backright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, // Local Feedback Source
				RobotMap.PID_PRIMARY, // PID Slot for Source [0, 1]
				RobotMap.kTimeoutMs); // Configuration Timeout
		drive = new MecanumDrive(frontleft, backleft, frontright, backright);

		// Initialize AHRS board
		try {
			/*
			 * Communicate w/ navX-MXP via the MXP SPI Bus. Alternatively: I2C.Port.kMXP,
			 * SerialPort.Port.kMXP or SerialPort.Port.kUSB See
			 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.
			 */
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}

		// Initialize pressure sensors
		hiPressureSensor = new AnalogInput(RobotMap.highPressureSensor);
		loPressureSensor = new AnalogInput(RobotMap.lowPressureSensor);

		cameras = NetworkTableInstance.getDefault().getTable("Vision");

		// Setup vision PID loops
		pid_strafe = new PIDController(1.0, 0.0, 0.0, new StrafeSource(), new StrafeOutput());
		pid_turn = new PIDController(1.0, 0.0, 0.0, new TurnSource(), new TurnOutput());

		pid_strafe.setAbsoluteTolerance(1.0);
		pid_turn.setAbsoluteTolerance(1.0);

		pid_strafe.setSetpoint(0.0);
		pid_turn.setSetpoint(0.0);
		pid_strafe.enable();
		pid_turn.enable();

		// Add Sendable data to dashboard
		SmartDashboard.putData("Front Left", frontleft);
		SmartDashboard.putData("Front Right", frontright);
		SmartDashboard.putData("Back Left", backleft);
		SmartDashboard.putData("Back Right", backright);
		SmartDashboard.putData("Hi Pressure", hiPressureSensor);
		SmartDashboard.putData("Lo Pressure", loPressureSensor);
	}

	@Override
	public void initDefaultCommand() {
		// Default command is Teleop
		setDefaultCommand(new ChassisDriveTeleop());
	}

	/**
	 * Get Hi and Lo pressure sensors in PSI
	 */
	public double getLoPressure() {
		return 250.0 * (loPressureSensor.getVoltage() / PRESSURE_SENSOR_INPUTVOLTAGE) - 25.0;
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / PRESSURE_SENSOR_INPUTVOLTAGE) - 25.0;
	}

	/*****************************************************************
	 * Drive routines
	 *****************************************************************/

	/**
	 * Main mecanum drive
	 */
	public void driveChassis(double x, double y, double r) {
		drive.driveCartesian(x, y, r, ahrs.getYaw());
	}

	/**
	 * Teleop Drive assumes resetting "front" of robot based on Panel/Cargo Using
	 * field oriented control may be preferred over resetting front
	 */
	public void driveTeleop() {
		double x = 0.0;
		double y = 0.0;
		double r = 0.0;

		switch (Robot.grabber.getMode()) {
		case CARGO:
			x = Robot.oi.getDriveX();
			y = -Robot.oi.getDriveY();
			break;

		case PANEL:
			x = -Robot.oi.getDriveX();
			y = Robot.oi.getDriveY();
			break;

		default:
		}

		r = Robot.oi.getDriveR();
		driveChassis(x, y, r);
	}

	// Vision stuff

	NetworkTable cameras;

	double current_strafe = 0;
	double current_turn = 0;

	PIDController pid_strafe;
	PIDController pid_turn;

	boolean rumble_enabled = false;

	/**
	 * Gets the camera subtable for the current mode.
	 */
	NetworkTable getCamera() {
		if (Robot.grabber.getMode() == MODE.CARGO) {
			return cameras.getSubTable("Front");
		} else {
			return cameras.getSubTable("Rear");
		}
	}

	class StrafeSource implements PIDSource {
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			if (getCamera().getEntry("Lock").getBoolean(false) == false) {
				return 0.0;
			}

			return getCamera().getEntry("Strafe").getDouble(0.0);
		}
	}

	class TurnSource implements PIDSource {
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			if (getCamera().getEntry("Lock").getBoolean(false) == false) {
				return 0.0;
			}

			return getCamera().getEntry("Turn").getDouble(0.0);
		}
	}

	class StrafeOutput implements PIDOutput {
		@Override
		public void pidWrite(double output) {
			current_strafe = output;
		}
	}

	class TurnOutput implements PIDOutput {
		@Override
		public void pidWrite(double output) {
			current_turn = output;
		}
	}

	/**
	 * Prepares PID loops for driving. In particular, resets PID loops, clears
	 * output variables, and re-enables PID loops.
	 */
	public void driveVisionStart() {
		pid_strafe.reset();
		pid_turn.reset();
		current_strafe = 0;
		current_turn = 0;
		pid_turn.enable();
		pid_strafe.enable();
	}

	public void enableRumble() {
		rumble_enabled = true;
	}

	public void disableRumble() {
		rumble_enabled = false;
	}

	public void doRumble() {
		if (rumble_enabled && getCamera().getEntry("Lock").getBoolean(false)) {
			double strafe = getCamera().getEntry("Strafe").getNumber(0.0).doubleValue();
			if (strafe < 5.0) {
				Robot.oi.setDriverRumble(RumbleType.kLeftRumble);
			}
			if (strafe > -5.0) {
				Robot.oi.setDriverRumble(RumbleType.kRightRumble);
			}
		} else {
			Robot.oi.resetDriverRumble(RumbleType.kLeftRumble);
			Robot.oi.resetDriverRumble(RumbleType.kRightRumble);
		}
	}

	/**
	 * Drives chassis motors using vision PID loops. NOTE: please call
	 * driveVisionStart() before using this.
	 */
	public void driveVision() {
		driveChassis(current_strafe, 0.0, current_turn);
	}

	public boolean visionFinished() {
		return getCamera().getEntry("Lock").getBoolean(false) && !pid_strafe.onTarget() && !pid_turn.onTarget();
	}

	/**
	 * Drive routines TBD or to be removed
	 */

	public void distFromBay() {
	}

	/**
	 * Detect "jerk" in direction of travel; Ignore being hit from side or behind
	 * 
	 */
	public void findJerk() {
		double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_Y = ahrs.getWorldLinearAccelY();
		double currentJerkY = curr_world_linear_accel_Y - last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_Y;

		if (Robot.grabber.getMode() == MODE.PANEL) {
			if (currentJerkX > kCollisionThreshold_DeltaG && Math.abs(currentJerkY) < 0.1) {
				collisionDetected = true;
			}
		}

		if (Robot.grabber.getMode() == MODE.CARGO) {
			if (currentJerkX < -kCollisionThreshold_DeltaG && Math.abs(currentJerkY) < 0.1) {
				collisionDetected = true;
			}
		}

	}

	public boolean isCollisionDetected() {
		return collisionDetected;
	}

	public void resetCollisionDetected() {
		collisionDetected = false;
	}

	/**
	 * Get AHRS info - Pitch, Roll, Yaw in degrees
	 */
	public double getPitch() {
		return ahrs.getRoll();
	}

	public double getRoll() {
		return ahrs.getPitch();
	}

	public double getYaw() {
		return ahrs.getYaw();
	}
}
