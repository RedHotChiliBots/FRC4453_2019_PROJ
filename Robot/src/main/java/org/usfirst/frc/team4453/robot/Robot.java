/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4453.robot;

import java.util.HashMap;

import org.usfirst.frc.team4453.robot.commands.autonomous.*;
import org.usfirst.frc.team4453.robot.library.Navigation;
//import org.usfirst.frc.team4453.robot.library.Vision;
import org.usfirst.frc.team4453.robot.subsystems.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

    public static Navigation navigation; 

    public static Chassis chassis;
    public static Climber climber;
    public static Grabber grabber;
    public static Shooter shooter;
    public static Wings   wings;
    public static Hook    hook;

    public static AHRS ahrs;

    public static OI oi;

    // public Vision vision;

    public enum RobotPosition {
        LEFT,
        RIGHT,
        CENTER
    }
    
    private SendableChooser<HashMap<RobotPosition, Command>> autoChooser = new SendableChooser<>();
    
    private HashMap<RobotPosition, Command> makeAutoChooserEntry(Command l, Command c, Command r)
    {
        HashMap<RobotPosition, Command> ret = new HashMap<RobotPosition, Command>();
        ret.put(RobotPosition.LEFT, l);
        ret.put(RobotPosition.CENTER, c);
        ret.put(RobotPosition.RIGHT, r);
        return ret;
    }
    
    private SendableChooser<RobotPosition> positionChooser = new SendableChooser<>();
    public RobotPosition getRobotStartingPosition()
    {
        return positionChooser.getSelected();
    }
    
    private Command autoCommand = null;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println("Robot Starting...");
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.getBoardYawAxis();
        ahrs.setSubsystem("Chassis");
        System.out.println("AHRS Started!");
        
        //vision = new Vision();
        //System.out.println("Vision Started!");

        navigation = new Navigation();
        System.out.println("Navigation constructed!");
        
        chassis = new Chassis();
        System.out.println("Chassis constructed!");
        climber = new Climber();
        System.out.println("Climber constructed!");
        grabber = new Grabber();
        System.out.println("Grabber constructed!");
        wings = new Wings();
        System.out.println("Wings constructed!");
        hook = new Hook();
        System.out.println("Hook constructed!");
        shooter = new Shooter();

        oi = new OI();
        System.out.println("OI constructed!");

        ahrs.zeroYaw();
        System.out.println("Yaw reset!");
        System.out.println("Robot started!");

        
        autoChooser.addDefault("No-Op", makeAutoChooserEntry(new NoOp(), new NoOp(), new NoOp()));
        autoChooser.addObject("Cross Auto Line", makeAutoChooserEntry(new CrossAutoLine(), new CrossAutoLine(), new CrossAutoLine()));
        autoChooser.addObject("Place Cube (Simple)", makeAutoChooserEntry(new ApproachAndPlaceCube(RobotPosition.LEFT), new ApproachAndPlaceCube(RobotPosition.CENTER), new ApproachAndPlaceCube(RobotPosition.RIGHT)));
        autoChooser.addObject("Any-positions Auto", makeAutoChooserEntry(new AutoAnyPosition(RobotPosition.LEFT), new AutoAnyPosition(RobotPosition.CENTER), new AutoAnyPosition(RobotPosition.RIGHT)));
        autoChooser.addObject("Test", makeAutoChooserEntry(new AutoTest(), new AutoTest(), new AutoTest()));
        autoChooser.addObject("Coord", makeAutoChooserEntry(new AutoCoord(), new AutoCoord(), new AutoCoord()));
        autoChooser.addObject("Place Cube (Complex)", makeAutoChooserEntry(new ApproachAndPlaceCube2(RobotPosition.LEFT), new ApproachAndPlaceCube2(RobotPosition.CENTER), new ApproachAndPlaceCube2(RobotPosition.RIGHT)));
        autoChooser.setName("Auto Command");
        SmartDashboard.putData(autoChooser);
        
        positionChooser.addObject("Right", RobotPosition.RIGHT);
        positionChooser.addDefault("Center", RobotPosition.CENTER);
        positionChooser.addObject("Left", RobotPosition.LEFT);
        positionChooser.setName("Robot Starting Position");
        SmartDashboard.putData(positionChooser);
        
    }

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
        grabber.init();
        //hook.init();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        telemetry();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        ahrs.zeroYaw();
        //grabber.init();
        //hook.init();
        autoCommand = autoChooser.getSelected().get(getRobotStartingPosition());

        // String autoSelected = SmartDashboard.getString("Auto Selector",
        // "Default");
        // switch (autoSelected) {
        // case "My Auto":
        // autonomousCommand = new MyAutoCommand();
        // break;
        // case "Default Auto":
        // default:
        // autonomousCommand = new ExampleCommand();
        // break;
        // }

        // schedule the autonomous command (example)
        if (autoCommand != null) {
            autoCommand.start();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        telemetry();
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
        telemetry();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        telemetry();
    }

    public void telemetry() {
        //SmartDashboard.putNumber("Vision distance", vision.getDistance());
        //SmartDashboard.putNumber("Vision angle", vision.getAngle());
        SmartDashboard.putNumber("Left Distance", chassis.getLeftDistance());
        SmartDashboard.putNumber("Right Distance", chassis.getRightDistance());
        SmartDashboard.putNumber("Hi Pressure", chassis.getHiPressure());
        SmartDashboard.putNumber("Lo Pressure", chassis.getLoPressure());
        SmartDashboard.putNumber("Heading", Robot.ahrs.getAngle());
        SmartDashboard.putNumber("Turn Rate", Robot.ahrs.getRate());
        SmartDashboard.putBoolean("Grabber Limit Hit", grabber.isLimitHit());
        SmartDashboard.putBoolean("Shooter Limit Hit", shooter.isLimitHit());
        SmartDashboard.putBoolean("Hook Limit Hit", hook.isLimitHit());
        SmartDashboard.putNumber("Tilt Angle", grabber.getTilt());
        //SmartDashboard.putNumber("Hook Height", hook.getDistance());
        SmartDashboard.putNumber("Left Chassis encoder", chassis.getLeftEncoder());
        SmartDashboard.putNumber("Right Chassis encoder", chassis.getRightEncoder());
    }
}
