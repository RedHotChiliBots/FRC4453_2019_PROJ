/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;


/**
 * Add your docs here.
 */
public class CANSparkMaxSendable extends CANSparkMax implements Sendable {

	private String name = "";
	private String subSystem = "";

	public CANSparkMaxSendable(int id, MotorType type) {
		super(id, type);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public void setName(String name) {
		this.name = name;
	}

	@Override
	public String getSubsystem() {
		return subSystem;
	}

	@Override
	public void setSubsystem(String subsystem) {
		this.subSystem = subsystem;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Speed Controller");
		builder.setSafeState(this::stopMotor);
		builder.addDoubleProperty("Value", this::get, this::set);

	}
	
}
