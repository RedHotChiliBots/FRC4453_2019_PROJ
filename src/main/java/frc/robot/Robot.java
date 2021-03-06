/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.commands.ChassisDriveJerk;
import frc.robot.commands.ChassisDriveTeleop;
import frc.robot.commands.LiftStartup;
import frc.robot.commands.LiftStop;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Lift;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;

  public static Chassis chassis;
  public static Grabber grabber;
  public static Lift lift;
  public static Climber climber;

  public static Preferences prefs = null;

  /**
   * This function is run when the robot is first started up and should be used
   * for initialization code.
   */
  @Override
  public void robotInit() {
    prefs = Preferences.getInstance();
    initPrefs();

    oi = new OI();
    grabber = new Grabber(); // MUST be before chassis because of vision
    chassis = new Chassis();
    lift = new Lift();
    climber = new Climber();
    oi.init();

    chassis.ahrs.zeroYaw();

    SmartDashboard.putData(Scheduler.getInstance());
    SmartDashboard.putData("DriveJerk", new ChassisDriveJerk());
    SmartDashboard.putData("DriveTeleop", new ChassisDriveTeleop());

    Robot.climber.retractback();
    Robot.climber.retractfront();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    telemetry();
    chassis.findJerk();
    chassis.doRumble();
    chassis.leds.set(ControlMode.PercentOutput, 0.5);
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    Robot.climber.retractback();
    Robot.climber.retractfront();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings and commands.
   */
  @Override
  public void autonomousInit() {
    chassis.ahrs.zeroYaw();
    Scheduler.getInstance().add(new LiftStartup());
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    chassis.ahrs.zeroYaw();
    Scheduler.getInstance().add(new LiftStop());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * 
   */
  private int i = 0;

  private void telemetry() {

    SmartDashboard.putData(chassis);
    SmartDashboard.putData(grabber);
    SmartDashboard.putData(lift);
    SmartDashboard.putData(climber);

    switch (i) {
    case 0:
      // SmartDashboard.putNumber("Heading", chassis.ahrs.getYaw());
      // SmartDashboard.putNumber("Turn Rate", chassis.ahrs.getRate());
      // SmartDashboard.putNumber("Pitch", chassis.ahrs.getRoll());
      SmartDashboard.putBoolean("Collision Detected", Robot.chassis.isCollisionDetected());
      SmartDashboard.putString("Mode", Robot.grabber.getMode().name());
      SmartDashboard.putString("Dir", Robot.grabber.getDir().name());
      SmartDashboard.putString("Level", Robot.lift.getLevel().name());
      SmartDashboard.putString("Action", Robot.grabber.getAction().name());
      i++;
      break;

    case 1:
      // if (Robot.lift.motor.getControlMode() == ControlMode.Position) {
      // SmartDashboard.putNumber("Lift Target",
      // Robot.lift.motor.getClosedLoopTarget());
      // }
      SmartDashboard.putNumber("Lift Target", Robot.lift.getTgtPosition());
      SmartDashboard.putNumber("Lift Current", Robot.lift.getMotorCurrent());
      SmartDashboard.putNumber("Lift Temp", Robot.lift.getMotorTemp());
      // SmartDashboard.putNumber("Lift Position",
      // Robot.lift.motor.getSelectedSensorPosition());
      // SmartDashboard.putNumber("Lift Velocity",
      // Robot.lift.motor.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Lift Position", Robot.lift.encoder.getPosition());
      SmartDashboard.putNumber("Lift Velocity", Robot.lift.encoder.getVelocity());
      SmartDashboard.putNumber("Panel Dist Sensor", Robot.chassis.getPanelDist());
      SmartDashboard.putNumber("Cargo Dist Sensor", Robot.chassis.getCargoDist());
      i++;
      break;

    case 2:
      SmartDashboard.putNumber("Lo Pressure", Robot.chassis.getLoPressure());
      SmartDashboard.putNumber("Hi Pressure", Robot.chassis.getHiPressure());
      SmartDashboard.putBoolean("Front Climb", Robot.climber.isFrontClimb());
      SmartDashboard.putBoolean("Back Climb", Robot.climber.isBackClimb());
      SmartDashboard.putBoolean("Front Step", Robot.climber.isFrontStep());
      SmartDashboard.putBoolean("Back Step", Robot.climber.isBackStep());
      SmartDashboard.putBoolean("isCPSensor", Robot.chassis.isCPSensor());
      i++;
      break;

    case 3:
      SmartDashboard.putNumber("Front Dist", Robot.climber.getDistFrontSensor());
      SmartDashboard.putNumber("Back Dist", Robot.climber.getDistBackSensor());
      SmartDashboard.putData("Climb Front", Robot.climber.climbFront);
      SmartDashboard.putData("Climb Back", Robot.climber.climbBack);
      SmartDashboard.putNumber("Grab L Current", Robot.grabber.getMotorLCurrent());
      SmartDashboard.putNumber("Grab R Current", Robot.grabber.getMotorRCurrent());
      SmartDashboard.putBoolean("Lift Rev Limit", Robot.lift.rLimit.get());
      i++;
      break;

    case 4:
      i = 0;
    }
  }

  private void initPrefs() {
    if (!prefs.containsKey("LiftMotorReset"))
      prefs.putDouble("LiftMotorReset", -1.0);
    if (!prefs.containsKey("CurrentThreshold"))
      prefs.putDouble("CurrentThreshold", 9.0);
    if (!prefs.containsKey("LiftPosError"))
      prefs.putDouble("LiftPosError", 5.0);
    if (!prefs.containsKey("LiftDownSpd"))
      prefs.putDouble("LiftDownSpd", 0.25);
    if (!prefs.containsKey("FStepDistHigh"))
      prefs.putDouble("FStepDistHigh", 10.0);
    if (!prefs.containsKey("BStepDistHigh"))
      prefs.putDouble("BStepDistHigh", 10.0);
    if (!prefs.containsKey("FStepAngleHigh"))
      prefs.putDouble("FStepAngleHigh", 18.0);
    if (!prefs.containsKey("FStepAngleLow"))
      prefs.putDouble("FStepAngleLow", 14.0);
    if (!prefs.containsKey("BStepAngleHigh"))
      prefs.putDouble("BStepAngleHigh", 2.0);
    if (!prefs.containsKey("BStepAngleLow"))
      prefs.putDouble("BStepAngleLow", -2.0);
    if (!prefs.containsKey("BStepDistLow"))
      prefs.putDouble("BStepDistLow", 4.0);
    if (!prefs.containsKey("FStepDistLow"))
      prefs.putDouble("FStepDistLow", 4.0);
    if (!prefs.containsKey("CargoGrabSpd"))
      prefs.putDouble("CargoGrabSpd", 0.5);
    if (!prefs.containsKey("CargoRelSpd"))
      prefs.putDouble("CargoRelSpd", 0.5);
    if (!prefs.containsKey("ChassisDistAllowedErr"))
      prefs.putDouble("ChassisDistAllowedErr", 10);
    if (!prefs.containsKey("GrabberMotorMaxCurrent"))
      prefs.putDouble("GrabberMotorMaxCurrent", 15);
    if (!prefs.containsKey("GrabberMotorMinCurrent"))
      prefs.putDouble("GrabberMotorMinCurrent", 5);
    if (!prefs.containsKey("Dist From Wall"))
      prefs.putDouble("Dist From Wall", 10);

    if (prefs.containsKey("LLMotorReset"))
      prefs.remove("LLMotorReset");
    if (prefs.containsKey("ULMotorReset"))
      prefs.remove("ULMotorReset");
  }
}
