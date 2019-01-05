/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4453.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    // USB Controller Ports
    public static final int FIRST_CONTROLLER  = 0;
    public static final int SECOND_CONTROLLER = 1;

    // XBox Controller Axis
    public static final int LEFT_X_AXIS	       = 0;
    public static final int LEFT_Y_AXIS	       = 1;
    public static final int LEFT_TRIGGER_AXIS  = 2;
    public static final int RIGHT_TRIGGER_AXIS = 3;
    public static final int RIGHT_X_AXIS       = 4;
    public static final int RIGHT_Y_AXIS       = 5;
    public static final int DPAD_X_AXIS	       = 6;
    public static final int DPAD_Y_AXIS	       = 7;

    // XBox Controller Buttons
    public static final int A_BUTTON	 = 1;
    public static final int B_BUTTON	 = 2;
    public static final int X_BUTTON	 = 3;
    public static final int Y_BUTTON	 = 4;
    public static final int LEFT_BUMPER	 = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BACK	 = 7;
    public static final int START	 = 8;
    public static final int LEFT_STICK	 = 9;
    public static final int RIGHT_STICK	 = 10;

    // Attack 3 Axis
    public static final int X_AXIS	  = 0;
    public static final int Y_AXIS	  = 1;
    public static final int THROTTLE_AXIS = 2;

    // Attack 3 Buttons
    public static final int TRIGGER_1 = 1;
    public static final int BUTTON_2  = 2;
    public static final int BUTTON_3  = 3;
    public static final int BUTTON_4  = 4;
    public static final int BUTTON_5  = 5;
    public static final int BUTTON_6  = 6;
    public static final int BUTTON_7  = 7;
    public static final int BUTTON_8  = 8;
    public static final int BUTTON_9  = 9;
    public static final int BUTTON_10 = 10;
    public static final int BUTTON_11 = 11;

    // ====== Constants ======
    public static final DoubleSolenoid.Value LIFTER_UP	 = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value LIFTER_DOWN = DoubleSolenoid.Value.kReverse;

    public static final DoubleSolenoid.Value SHIFTER_HIGH_GEAR = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value SHIFTER_LOW_GEAR  = DoubleSolenoid.Value.kReverse;

    public static final boolean	HIGH_GEAR = true;
    public static final boolean	LOW_GEAR  = false;

    // ====== Motor Controllers ======
    // Chassis
    public static final int CHASSIS_FRONT_RIGHT_MOTOR = 1; // TODO Verify L/R Motor
    public static final int CHASSIS_FRONT_LEFT_MOTOR  = 0; // TODO Verify L/R Motor
    public static final int CHASSIS_MID_RIGHT_MOTOR   = 3; // TODO Verify L/R Motor
    public static final int CHASSIS_MID_LEFT_MOTOR    = 2; // TODO Verify L/R Motor
    public static final int CHASSIS_REAR_RIGHT_MOTOR  = 5; // TODO Verify L/R Motor
    public static final int CHASSIS_REAR_LEFT_MOTOR   = 4; // TODO Verify L/R Motor
    // Climber
    public static final int CLIMBER_RIGHT_MOTOR	= 6;
    public static final int CLIMBER_LEFT_MOTOR	= 7;
    public static final int CLIMBER_HOOK_MOTOR	= 11;
    // Grabber
    public static final int GRABBER_LEFT_MOTOR	= 8;
    public static final int GRABBER_RIGHT_MOTOR	= 9;
    public static final int GRABBER_TILT_MOTOR	= 10;
    // Shooter
    public static final int SHOOTER_WIND_MOTOR = 99;

    // ====== Solenoids ======
    // Chassis
    public static final int SHIFTER_HI_SOLENOID	= 0;
    public static final int SHIFTER_LO_SOLENOID	= 1;
    // Lifter
    public static final int LIFTER_UP_SOLENOID	 = 2;
    public static final int LIFTER_DOWN_SOLENOID = 3;
    // Grabber
    public static final int GRABBER_GRIP_SOLENOID    = 4;
    public static final int GRABBER_RELEASE_SOLENOID = 5;
    // Shooter
    public static final int SHOOTER_LATCH_SOLENOID   = 6;
    public static final int SHOOTER_RELEASE_SOLENOID = 7;

    // ====== Analog Inputs ======
    // Pressure Sensor
    public static final int HI_PRESSURE_SENSOR = 0;
    public static final int LO_PRESSURE_SENSOR = 1;
    // Distance Sensor
    public static final int LEFT_DISTANCE_SENSOR  = 2;
    public static final int RIGHT_DISTANCE_SENSOR = 3;

    // ====== Digital Inputs ======
    public static final int SHOOTER_LIMIT_SWITCH = 0;

    // ====== Relays ======
    public static final int SHOOTER_CLUTCH_RELAY = 0;

}
