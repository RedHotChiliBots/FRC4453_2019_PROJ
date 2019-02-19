/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

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
   * This function is run when the robot is first started up and should be used
   * for initialization code.
   */
  @Override
  public void robotInit() {
    prefs = Preferences.getInstance();
    initPrefs();

    oi = new OI();
    chassis = new Chassis();
    cargo = new CargoGrabber();
    uLift = new UpperLift();
    lLift = new LowerLift();
    panel = new PanelGrabber();
    climber = new Climber();
    oi.init();

    chassis.ahrs.zeroYaw();

    SmartDashboard.putData(Scheduler.getInstance());
    SmartDashboard.putData(chassis);
    SmartDashboard.putData(cargo);
    SmartDashboard.putData(uLift);
    SmartDashboard.putData(lLift);
    SmartDashboard.putData(panel);
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
      SmartDashboard.putBoolean("Collision Detected", Robot.chassis.IsCollisionDetected());
      SmartDashboard.putString("Mode", Robot.chassis.getMode().name());
      SmartDashboard.putString("Level", Robot.chassis.getLevel().name());
      i++;
      break;

    case 1:
      if (Robot.lLift.motor1.getControlMode() == ControlMode.Position) {
        SmartDashboard.putNumber("LLMotor1Tgt", Robot.lLift.motor1.getClosedLoopTarget());
      }
      SmartDashboard.putNumber("LL1Current", Robot.lLift.motor1.getOutputCurrent());
      SmartDashboard.putNumber("LLMotor1Pos", Robot.lLift.motor1.getSelectedSensorPosition());
      SmartDashboard.putNumber("LLMotor1Vel", Robot.lLift.motor1.getSelectedSensorVelocity());
      if (Robot.lLift.motor2.getControlMode() == ControlMode.Position) {
        SmartDashboard.putNumber("LLMotor2Tgt", Robot.lLift.motor2.getClosedLoopTarget());
      }
      SmartDashboard.putNumber("LL2Current", Robot.lLift.motor2.getOutputCurrent());
      SmartDashboard.putNumber("LLMotor2Pos", Robot.lLift.motor2.getSelectedSensorPosition());
      SmartDashboard.putNumber("LLMotor2Vel", Robot.lLift.motor2.getSelectedSensorVelocity());
      i++;
      break;

    case 2:
      if (Robot.uLift.motor1.getControlMode() == ControlMode.Position) {
        SmartDashboard.putNumber("ULMotor1Tgt", Robot.uLift.motor1.getClosedLoopTarget());
      }
      SmartDashboard.putNumber("UL1Current", Robot.uLift.motor1.getOutputCurrent());
      SmartDashboard.putNumber("ULMotor1Pos", Robot.uLift.motor1.getSelectedSensorPosition());
      SmartDashboard.putNumber("ULMotor1Vel", Robot.uLift.motor1.getSelectedSensorVelocity());
      if (Robot.uLift.motor2.getControlMode() == ControlMode.Position) {
        SmartDashboard.putNumber("ULMotor2Tgt", Robot.uLift.motor2.getClosedLoopTarget());
      }
      SmartDashboard.putNumber("UL2Current", Robot.uLift.motor2.getOutputCurrent());
      SmartDashboard.putNumber("ULMotor2Pos", Robot.uLift.motor2.getSelectedSensorPosition());
      SmartDashboard.putNumber("ULMotor2Vel", Robot.uLift.motor2.getSelectedSensorVelocity());
      i++;
      break;

    case 3:
      SmartDashboard.putNumber("Lo Pressure", Robot.chassis.getLoPressure());
      SmartDashboard.putNumber("Hi Pressure", Robot.chassis.getHiPressure());
      SmartDashboard.putBoolean("Front Climb", Robot.climber.isFrontClimb());
      SmartDashboard.putBoolean("Back Climb", Robot.climber.isBackClimb());
      i++;
      break;

    case 4:
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
