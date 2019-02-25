/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.ChassisDriveJerk;
import frc.robot.commands.ChassisDriveTeleop;

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
    chassis = new Chassis();
    grabber = new Grabber();
    lift = new Lift();
    climber = new Climber();
    oi.init();

    chassis.ahrs.zeroYaw();

    SmartDashboard.putData(Scheduler.getInstance());
    SmartDashboard.putData(chassis);
    SmartDashboard.putData(grabber);
    SmartDashboard.putData(lift);
    SmartDashboard.putData(climber);
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
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    chassis.ahrs.zeroYaw();
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
    switch (i) {
    case 0:
      SmartDashboard.putNumber("Heading", chassis.ahrs.getYaw());
      SmartDashboard.putNumber("Turn Rate", chassis.ahrs.getRate());
      SmartDashboard.putNumber("Pitch", chassis.ahrs.getRoll());
      SmartDashboard.putBoolean("Collision Detected", Robot.chassis.isCollisionDetected());
      SmartDashboard.putString("Mode", Robot.grabber.getMode().name());
      SmartDashboard.putString("Level", Robot.lift.getLevel().name());
      i++;
      break;

    case 1:
      if (Robot.lift.motor.getControlMode() == ControlMode.Position) {
        SmartDashboard.putNumber("Lift Target", Robot.lift.motor.getClosedLoopTarget());
      }
      SmartDashboard.putNumber("Lift Current", Robot.lift.motor.getOutputCurrent());
      SmartDashboard.putNumber("Lift Position", Robot.lift.motor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Lift Velocity", Robot.lift.motor.getSelectedSensorVelocity());
      i++;
      break;

    case 2:
      SmartDashboard.putNumber("Lo Pressure", Robot.chassis.getLoPressure());
      SmartDashboard.putNumber("Hi Pressure", Robot.chassis.getHiPressure());
      SmartDashboard.putBoolean("Front Climb", Robot.climber.isFrontClimb());
      SmartDashboard.putBoolean("Back Climb", Robot.climber.isBackClimb());
      i++;
      break;

    case 3:
      SmartDashboard.putBoolean("Front Step", Robot.climber.isFrontStep());
      SmartDashboard.putBoolean("Back Step", Robot.climber.isBackStep());
      SmartDashboard.putNumber("Front Dist", Robot.climber.getDistFrontSensor());
      SmartDashboard.putNumber("Back Dist", Robot.climber.getDistBackSensor());
      i = 0;
    }
  }

  private void initPrefs() {
    if (!prefs.containsKey("LLMotorReset"))
      prefs.putDouble("LLMotorReset", -1.0);
    if (!prefs.containsKey("ULMotorReset"))
      prefs.putDouble("ULMotorReset", -1.0);
    if (!prefs.containsKey("CurrentThreshold"))
      prefs.putDouble("CurrentThreshold", 9.0);
    if (!prefs.containsKey("LiftPosError"))
      prefs.putDouble("LiftPosError", 5.0);
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
      prefs.getDouble("CargoGrabSpd", 0.5);
    if (!prefs.containsKey("CargoRelSpd"))
      prefs.getDouble("CargoRelSpd", 0.5);
    if (!prefs.containsKey("ChassisDistAllowedErr"))
      prefs.getDouble("ChassisDistAllowedErr", 10);

    if (prefs.containsKey("BackStepAngleHigh"))
      prefs.remove("BackStepAngleHigh");
    if (prefs.containsKey("BackStepAngleLow"))
      prefs.remove("BackStepAngleLow");
    if (prefs.containsKey("FrontStepAngleHigh"))
      prefs.remove("FrontStepAngleHigh");
    if (prefs.containsKey("FrontStepAngleLow"))
      prefs.remove("FrontStepAngleLow");
    if (prefs.containsKey("BackStepDist"))
      prefs.remove("BackStepDist");
    if (prefs.containsKey("FrontStepDist"))
      prefs.remove("FrontStepDist");
    if (prefs.containsKey("BackStepDistHigh"))
      prefs.remove("BackStepDistHigh");
    if (prefs.containsKey("FrontStepAngle"))
      prefs.remove("FrontStepAngle");
    if (prefs.containsKey("BackStepAngle"))
      prefs.remove("BackStepAngle");
    if (prefs.containsKey("LMotorReset"))
      prefs.remove("LMotorReset");
  }
}
