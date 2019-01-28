/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

//import com.sun.tools.javac.comp.Lower;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ChassisDriveJerk;
import frc.robot.commands.ChassisDriveTeleop;
import frc.robot.subsystems.CargoGrabber;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LowerLift;
import frc.robot.subsystems.PanelGrabber;
import frc.robot.subsystems.UpperLift;

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
  public static CargoGrabber cargo;
  public static UpperLift uLift;
  public static LowerLift lLift;
  public static PanelGrabber panel;
  public static Climber climber;

  public static Preferences prefs = null;


  /**
   * This function is run when the robot is first started up and should be
   * used for initialization code.
   */
  @Override
  public void robotInit() {
    prefs = Preferences.getInstance();

    oi = new OI();
    chassis = new Chassis();
//    cargo = new CargoGrabber();
//    uLift = new UpperLift();
    lLift = new LowerLift();
//    panel = new PanelGrabber();
//    climber = new Climber();
    oi.init();

    chassis.ahrs.zeroYaw();

    SmartDashboard.putData(Scheduler.getInstance());
    SmartDashboard.putData(chassis);
    SmartDashboard.putData("DriveJerk", new ChassisDriveJerk());
    SmartDashboard.putData("DriveTeleop", new ChassisDriveTeleop());
    SmartDashboard.putString("Mode", Robot.chassis.mode==Chassis.Mode.PANEL?"PANEL":"CARGO");

    initPrefs();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    telemetry();
    chassis.findJerk();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
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

  private void telemetry() {
    SmartDashboard.putNumber("Heading", chassis.ahrs.getYaw());
    SmartDashboard.putNumber("Turn Rate", chassis.ahrs.getRate()); 
    SmartDashboard.putBoolean("Collision Detected", Robot.chassis.IsCollisionDetected()); 
//    SmartDashboard.putBoolean("Panel Selected", Robot.chassis.isPanelSelected());
//    SmartDashboard.putBoolean("Cargo Selected", Robot.chassis.isCargoSelected());
    SmartDashboard.putString("Mode", Robot.chassis.mode==Chassis.Mode.PANEL?"PANEL":"CARGO");
    SmartDashboard.putNumber("Current", Robot.lLift.motor1.getOutputCurrent());
  }

  private void initPrefs() {
    if (prefs.containsKey("LLMotorReset")) prefs.putDouble("LLMotorReset", -1.0);
    if (prefs.containsKey("CurrentThreshold")) prefs.putDouble("CurrentThreshold", 9.0);
    if (prefs.containsKey("LiftPosError")) prefs.putDouble("LiftPosError", 5.0);
  }
}
