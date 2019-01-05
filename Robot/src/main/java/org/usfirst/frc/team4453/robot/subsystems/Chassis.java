package org.usfirst.frc.team4453.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc.team4453.robot.Robot;
import org.usfirst.frc.team4453.robot.RobotMap;
import org.usfirst.frc.team4453.robot.commands.TeleopDrive;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Chassis extends PIDSubsystem {
    private static final double CHASSIS_3RD_GEAR_RATIO = 2.5; // Vex 3CIM Ball Shifter 3rd Stage 24:60
    private static final double CHASSIS_ENCODER_RATIO = 3.0; // Encoder gear ratio 1:3
    public static final double CHASSIS_ENCODER_TICKS_PER_REVOLUTION = 4096; // SRX Mag Encoder 1024cpr Quadrature
    private static final double CHASSIS_GEARBOX2ENCODER_RATIO = CHASSIS_3RD_GEAR_RATIO * CHASSIS_ENCODER_RATIO
            * CHASSIS_ENCODER_TICKS_PER_REVOLUTION;
    public static final double CHASSIS_WHEEL_DIAMETER = 6; // inches
    private static final double CHASSIS_TICKS_PER_INCH = CHASSIS_GEARBOX2ENCODER_RATIO
            / (CHASSIS_WHEEL_DIAMETER * Math.PI);
    
    private static final double FEET_TO_METERS = 3.28084;

    public static final double MAX_VELOCITY     = 15.0 * FEET_TO_METERS;
    public static final double MAX_ACCELERATION = MAX_VELOCITY / 0.5;
    public static final double MAX_JERK         = MAX_ACCELERATION / 0.1;


    private final WPI_TalonSRX leftFront  = new WPI_TalonSRX(RobotMap.CHASSIS_FRONT_LEFT_MOTOR);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(RobotMap.CHASSIS_FRONT_RIGHT_MOTOR);
    private final WPI_TalonSRX leftMid    = new WPI_TalonSRX(RobotMap.CHASSIS_MID_LEFT_MOTOR);
    private final WPI_TalonSRX rightMid   = new WPI_TalonSRX(RobotMap.CHASSIS_MID_RIGHT_MOTOR);
    private final WPI_TalonSRX leftBack   = new WPI_TalonSRX(RobotMap.CHASSIS_REAR_LEFT_MOTOR);
    private final WPI_TalonSRX rightBack  = new WPI_TalonSRX(RobotMap.CHASSIS_REAR_RIGHT_MOTOR);

    private final DoubleSolenoid shifter  = new DoubleSolenoid(RobotMap.SHIFTER_HI_SOLENOID, RobotMap.SHIFTER_LO_SOLENOID);

    private final DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

    // Constants
    private final double PRESSURE_SENSOR_INPUTVOLTAGE = 5.0;
    private final double DISTANCE_SENSOR_SCALE        = 5.0 / 512;

    // Pressure sensors
    private AnalogInput hiPressureSensor    = new AnalogInput(RobotMap.HI_PRESSURE_SENSOR);
    private AnalogInput loPressureSensor    = new AnalogInput(RobotMap.LO_PRESSURE_SENSOR);
    // Distance Sensors
    private AnalogInput leftDistanceSensor  = new AnalogInput(RobotMap.LEFT_DISTANCE_SENSOR);
    private AnalogInput rightDistanceSensor = new AnalogInput(RobotMap.RIGHT_DISTANCE_SENSOR);

    private Compressor  compressor          = new Compressor();

    private double heading = 0.0;
    private double xPos    = 0.0;
    private double yPos    = 0.0;
    
    private double PIDSpeed = 0;
    
    private PIDSource distancePIDInput = new PIDSource() {
        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            return (leftFront.getSelectedSensorPosition(0) + rightFront.getSelectedSensorPosition(0))/2.0; 
        }
    };

    private PIDOutput distancePIDOutput = new PIDOutput() {
        @Override
        public void pidWrite(double output) {
            PIDSpeed = output;
        }
    };
    
    private PIDController distancePID = new PIDController(0.00005, 0.0000009, 0.0007, distancePIDInput, distancePIDOutput); // TODO: PID Values
    
    public Chassis() {
        super("Chassis", 0.00001, 0.0, 0.0); // TODO: PID Values
        System.out.println("Entering Chassis...");
        
        System.out.println("Configuring Distance PID...");
        getPIDController().setInputRange(0, 360);
        getPIDController().setContinuous();
        getPIDController().setAbsoluteTolerance(0.2); // TODO
        getPIDController().setOutputRange(-.6, .6);
        distancePID.setAbsoluteTolerance(0.5); // TODO
        distancePID.setOutputRange(-.6, .6);
        distancePID.setName("Chassis", "Distance PID");
        SmartDashboard.putData(distancePID);
        System.out.println("Distance PID configured!");
        
        System.out.println("Configuring left motors...");
        leftFront.setSubsystem("Chassis");
        leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
        leftFront.setSensorPhase(true);
        leftMid.follow(leftFront);
        leftMid.setSubsystem("Chassis");
        leftBack.follow(leftFront);
        leftBack.setSubsystem("Chassis");
        System.out.println("Left motors configured!");

        System.out.println("Configuring right motors...");
        rightFront.setSubsystem("Chassis");
        rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
        rightFront.setSensorPhase(false);
        rightMid.follow(rightFront);
        rightMid.setSubsystem("Chassis");
        rightBack.follow(rightFront);
        rightBack.setSubsystem("Chassis");
        System.out.println("Right motors configured!");

        System.out.println("Enabling compressor...");
        compressor.setSubsystem("Chassis");
        compressor.start();
        System.out.println("Compressor enabled!");

        System.out.println("Misc. config starting...");
        hiPressureSensor.setSubsystem("Chassis");
        loPressureSensor.setSubsystem("Chassis");
        leftDistanceSensor.setSubsystem("Chassis");
        rightDistanceSensor.setSubsystem("Chassis");

        drive.setSubsystem("Chassis");

        shifter.setSubsystem("Chassis");
        System.out.println("Misc. config finished!");
        
        System.out.println("Disabling angle PID...");
        disable();
        System.out.println("Disabled angle PID!");
        System.out.println("Disabling distance PID...");
        distancePID.disable();
        System.out.println("Disabled distance PID!");

        System.out.println("Shifting low...");
        shift(false);
        System.out.println("Shifted low!");
        System.out.println("Exiting Chassis...");
    }
    
    public void drive(double lspeed, double rspeed) {
        drive.tankDrive(lspeed, rspeed);
    }

    public void curveDrive(double spdCmd, double rotCmd, boolean quickTurn) {
        getPIDController().disable();
        drive.curvatureDrive(spdCmd, rotCmd, quickTurn);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new TeleopDrive());
    }

    public void shift(boolean highgear) {
        shifter.set(highgear ? RobotMap.SHIFTER_HIGH_GEAR : RobotMap.SHIFTER_LOW_GEAR);
    }

    public void stop() {
        PIDSpeed = 0;
        getPIDController().reset();
        distancePID.reset();
        drive.stopMotor();
    }

    public double getHeading() {
        return heading;
    }
    
    public void setHeading(double h) {
        heading = h;
    }
    
    public double getXPos() {
        return xPos;
    }
    
    public void setXPos(double x) {
        xPos = x;
    }
    
    public double getYPos() {
        return yPos;
    }
    
    public void setYPos(double y) {
        yPos = y;
    }
    
    public void resetNavigation() {
        xPos = 0;
        yPos = 0;
        heading = 0;
    }

    public double getLoPressure() {
        return 250.0 * (loPressureSensor.getVoltage() / PRESSURE_SENSOR_INPUTVOLTAGE) - 25.0; // ToDo
    }

    public double getHiPressure() {
        return 250.0 * (hiPressureSensor.getVoltage() / PRESSURE_SENSOR_INPUTVOLTAGE) - 25.0;
    }

    public double getLeftDistance() {
        return leftDistanceSensor.getVoltage() / DISTANCE_SENSOR_SCALE;
    }

    public double getRightDistance() {
        return rightDistanceSensor.getVoltage() / DISTANCE_SENSOR_SCALE;
    }

    public void arcadeDrive(double spdAxis, double turnAxis) {
        //getPIDController().disable();
        drive.arcadeDrive(spdAxis, turnAxis);
    }

    public void turn(double angle) {
        driveWithHeading(1.0, angle);	//testing Larsen from 0 to 1
        //TODO Larsen
    }
    
    public void driveWithHeading(double speed, double angle) {
        Robot.ahrs.zeroYaw();
        getPIDController().reset();
        getPIDController().enable();
        distancePID.disable();
        setSetpoint(angle);
        PIDSpeed = speed;
    }
    
    public void driveDistanceWithHeading(double distance, double angle)
    {
        Robot.ahrs.zeroYaw();
        leftFront.setSelectedSensorPosition(0, 0, 100);
        rightFront.setSelectedSensorPosition(0, 0, 100);
        distancePID.setSetpoint(distance * CHASSIS_TICKS_PER_INCH);
        setSetpoint(angle);
        getPIDController().reset();
        getPIDController().enable();
        distancePID.reset();
        distancePID.enable();
    }
    
    public void driveDistance(double distance)
    {
        Robot.ahrs.zeroYaw();
        leftFront.setSelectedSensorPosition(0, 0, 100);
        rightFront.setSelectedSensorPosition(0, 0, 100);
        distancePID.setSetpoint(distance * CHASSIS_TICKS_PER_INCH);
        setSetpoint(0.0);
        getPIDController().reset();
        getPIDController().enable();
        distancePID.reset();
        distancePID.enable();
    }
    
    public boolean distanceOnTarget() {
        return Math.abs(distancePID.getSetpoint() - distancePIDInput.pidGet()) < 1.0*CHASSIS_TICKS_PER_INCH && Math.abs(chassisSpeed()) < .5;
    }
    
    public double chassisSpeed() {
        return (leftFront.getSelectedSensorVelocity(0) + rightFront.getSelectedSensorVelocity(0)) * 10.0 / (2.0 * CHASSIS_TICKS_PER_INCH);
    }
    
    public double getLeftEncoder()
    {
        return leftFront.getSelectedSensorPosition(0) / CHASSIS_TICKS_PER_INCH;
    }
    
    public double getRightEncoder()
    {
        return rightFront.getSelectedSensorPosition(0) / CHASSIS_TICKS_PER_INCH;
    }

    public void driveRaw(double left, double right) {
        leftFront.set(left);
        rightFront.set(right);
    }
    
    public boolean angleOnTarget() {
        return getPIDController().onTarget() && Math.abs(Robot.ahrs.getRate()) < .5;
    }
    
    @Override
    protected double returnPIDInput() {
        return Robot.ahrs.getYaw() % 360;
    }

    @Override
    protected void usePIDOutput(double output) {
        //System.out.println("Chassis pid output speed=" + PIDSpeed + ", angle=" + output);
        drive.arcadeDrive(PIDSpeed, output);
        //drive.arcadeDrive(PIDSpeed, 0.0);
    }
}
