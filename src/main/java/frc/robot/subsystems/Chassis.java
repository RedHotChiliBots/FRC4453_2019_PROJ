/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import frc.robot.CANSparkMaxSendable;
import com.revrobotics.ControlType;

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
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.KID;
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
	// private WPI_TalonSRX frontleft = null;
	// private WPI_TalonSRX frontright = null;
	// private WPI_TalonSRX backleft = null;
	// private WPI_TalonSRX backright = null;

	private CANSparkMaxSendable frontleft = null;
	private CANSparkMaxSendable frontright = null;
	private CANSparkMaxSendable backleft = null;
	private CANSparkMaxSendable backright = null;

	public WPI_TalonSRX leds = null;

	public CANPIDController pidFL = null;
	public CANPIDController pidFR = null;
	public CANPIDController pidBL = null;
	public CANPIDController pidBR = null;

	private MecanumDrive drive = null;
	// private DifferentialDrive drive = null;

	// "jerk" detection
	private boolean collisionDetected = false;

	private double last_world_linear_accel_x = 0.0;
	private double last_world_linear_accel_y = 0.0;

	private final double kCollisionThreshold_DeltaG = 0.5;

	// Define navX board
	public AHRS ahrs = null;

	public KID kid = null;

	// Hi & Lo Pressure Sensors
	private AnalogInput hiPressureSensor = null;
	private AnalogInput loPressureSensor = null;

	// Lift Distance sensor
	private AnalogInput panelDist = null;
	private AnalogInput cargoDist = null;

	// Camera servos
	private Servo cargoServo = null;
	private Servo panelServo = null;

	private double cargoServoPos = 0.0;
	private double panelServoPos = 0.0;

	public Chassis() {
		super("Chassis");
		// Initialize drive train for Mechanam
		frontleft = new CANSparkMaxSendable(RobotMap.frontLeftMotor, CANSparkMax.MotorType.kBrushless);
		frontright = new CANSparkMaxSendable(RobotMap.frontRightMotor, CANSparkMax.MotorType.kBrushless);
		backleft = new CANSparkMaxSendable(RobotMap.backLeftMotor, CANSparkMax.MotorType.kBrushless);
		backright = new CANSparkMaxSendable(RobotMap.backRightMotor, CANSparkMax.MotorType.kBrushless);

		frontleft.restoreFactoryDefaults();
		frontleft.stopMotor();
		// frontleft.getPIDController().setReference(0.0, ControlType.kVoltage,
		// RobotMap.kSlot_Position);
		frontleft.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pidFL = frontleft.getPIDController();
		pidFL.setReference(0.0, ControlType.kDutyCycle, RobotMap.kSlot_Position);

		frontright.restoreFactoryDefaults();
		frontright.stopMotor();
		// frontright.getPIDController().setReference(0.0, ControlType.kVoltage,
		// RobotMap.kSlot_Position);
		frontright.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pidFR = frontleft.getPIDController();
		pidFR.setReference(0.0, ControlType.kDutyCycle, RobotMap.kSlot_Position);

		backleft.restoreFactoryDefaults();
		backleft.stopMotor();
		// backleft.getPIDController().setReference(0.0, ControlType.kVoltage,
		// RobotMap.kSlot_Position);
		backleft.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pidBL = frontleft.getPIDController();
		pidBL.setReference(0.0, ControlType.kDutyCycle, RobotMap.kSlot_Position);

		backright.restoreFactoryDefaults();
		backright.stopMotor();
		// backright.getPIDController().setReference(0.0, ControlType.kVoltage,
		// RobotMap.kSlot_Position);
		backright.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pidBR = frontleft.getPIDController();
		pidBR.setReference(0.0, ControlType.kDutyCycle, RobotMap.kSlot_Position);

		drive = new MecanumDrive(frontleft, backleft, frontright, backright);
		// drive = new DifferentialDrive(frontleft, frontright);

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

		// Initialize distance sensors
		panelDist = new AnalogInput(RobotMap.panelDistSensor);
		cargoDist = new AnalogInput(RobotMap.cargoDistSensor);

		// Initialize camera servos
		cargoServo = new Servo(RobotMap.cargoServo);
		panelServo = new Servo(RobotMap.panelServo);
		// Initialize camera position to forward looking (0.0 deg)
		setCargoServo(0.0);
		setPanelServo(0.0);

		kid = KID.OFF;

		// Initialize network table - cameras
		cameras = NetworkTableInstance.getDefault().getTable("Vision");

		// Setup vision PID loops
		pid_strafe = new PIDController(0.4, 0.01, 0.0, new StrafeSource(), new StrafeOutput());
		pid_turn = new PIDController(0.02, 0.0, 0.0, new TurnSource(), new TurnOutput());

		pid_strafe.setAbsoluteTolerance(1.0);
		pid_turn.setAbsoluteTolerance(1.0);

		pid_strafe.setOutputRange(-0.5, 0.5);
		pid_turn.setOutputRange(-0.25, 0.25);

		pid_strafe.setSetpoint(0.0);
		pid_turn.setSetpoint(0.0);
		pid_strafe.enable();
		pid_turn.enable();

		// Add Sendable data to dashboard

		leds = new WPI_TalonSRX(RobotMap.leds);

		leds.enableVoltageCompensation(true);
		leds.set(ControlMode.PercentOutput, 0.5);

		frontleft.setSubsystem("Chassis");
		backleft.setSubsystem("Chassis");
		frontright.setSubsystem("Chassis");
		backright.setSubsystem("Chassis");

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

	private final double SENSOR_INPUTVOLTAGE = 5.0;
	private final double DISTANCE_SENSOR_SCALE = 5.0 / 1024;

	/**
	 * Get Hi and Lo pressure sensors in PSI
	 */
	public double getLoPressure() {
		return 250.0 * (loPressureSensor.getVoltage() / SENSOR_INPUTVOLTAGE) - 25.0;
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / SENSOR_INPUTVOLTAGE) - 25.0;
	}

	/**
	 * Get Panel and Cargo distance sensor in inches
	 */
	// [5*(Vm/Vi)=Ri]
	public double getPanelDist() {
		return (panelDist.getVoltage() / DISTANCE_SENSOR_SCALE) * 5.0 / 25.4;
	}

	public double getCargoDist() {
		return (cargoDist.getVoltage() / DISTANCE_SENSOR_SCALE) * 5.0 / 25.4;
	}

	/**
	 * Set / get servo position servo angle is 0-170deg cmd angle is +-85deg
	 */
	private final double calibOffset = 0.0; // add/sub to calibrate camera at zero degrees
	private final double angleOffset = 85.0 + calibOffset;

	public void setCargoServo(double angle) {
		this.cargoServoPos = angle;
		cargoServo.setAngle(angle + angleOffset);
	}

	public double getCargoServo() {
		return this.cargoServoPos - angleOffset;
	}

	public void setPanelServo(double angle) {
		this.panelServoPos = angle;
		panelServo.setAngle(angle + angleOffset);
	}

	public double getPanelServo() {
		return this.panelServoPos - angleOffset;
	}

	/*****************************************************************
	 * Drive routines
	 *****************************************************************/

	/**
	 * Main mecanum drive
	 */
	public void driveChassis(double x, double y, double r) {
		drive.driveCartesian(x, y, r, ahrs.getYaw());
		// drive.tankDrive(-Robot.oi.getDriveLY(), -Robot.oi.getDriverRY());
	}

	public void driveChassisLocal(double x, double y, double r) {
		drive.driveCartesian(x, y, r);
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
			y = Robot.oi.getDriveY();
			break;

		case PANEL:
			x = -Robot.oi.getDriveX();
			y = -Robot.oi.getDriveY();
			break;

		default:
		}

		// x = -Robot.oi.getDriveX();
		// y = -Robot.oi.getDriveY();
		r = Robot.oi.getDriveR();
		driveChassisLocal(x, -y, r);
		// driveChassis(x, y);
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
	public void driveVision(double fwdVel) {
		if (Robot.grabber.getMode() == MODE.CARGO) {
			// drive.driveCartesian(-current_strafe, -fwdVel, -current_turn);
			// driveChassis(0.0, current_strafe,-current_turn);
		} else {
			// drive.driveCartesian(current_strafe, fwdVel, current_turn);
			// driveChassis(0.0, -current_strafe, current_turn);
		}
	}

	public boolean visionFinished() {
		return !getCamera().getEntry("Lock").getBoolean(false);
	}

	/**
	 * Drive routines TBD or to be removed
	 */

	public void distFromBay() {
	}

	/**
	 * Tracks target by reading angle from vision code via network tables and
	 * commands serve to angle
	 */
	public void trackTargets() {
		double angle = 0.0;
		if (cameras.getSubTable("Front").getEntry("Lock").getBoolean(false)) {
			angle = getCargoServo() + cameras.getSubTable("Front").getEntry("ServoError").getNumber(0.0).doubleValue();
		} else {
			angle = 0.0;
		}
		setCargoServo(angle);

		if (cameras.getSubTable("Front").getEntry("Lock").getBoolean(false)) {
			angle = getCargoServo() + cameras.getSubTable("Rear").getEntry("ServoError").getNumber(0.0).doubleValue();
		} else {
			angle = 0.0;
		}
		setPanelServo(angle);
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

	public boolean isCPSensor() {
		if (Robot.grabber.mode == MODE.CARGO) {
			return getCargoDist() < Robot.prefs.getDouble("Dist From Wall", 10);
		} else {
			return getPanelDist() < Robot.prefs.getDouble("Dist From Wall", 10);
		}
	}

	public void setKidMode(KID k) {
		kid = k;
	}

	public KID getKidMode() {
		return kid;
	}
}
