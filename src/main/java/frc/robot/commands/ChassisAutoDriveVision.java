/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.MODE;

public class ChassisAutoDriveVision extends Command {

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
      if (camera.getEntry("Lock").getBoolean(false) == false) {
        return 0.0;
      }

      return camera.getEntry("Strafe").getDouble(0.0);
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
      if (camera.getEntry("Lock").getBoolean(false) == false) {
        return 0.0;
      }

      return camera.getEntry("Turn").getDouble(0.0);
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

  NetworkTable camera;

  double current_strafe = 0;
  double current_turn = 0;

  PIDController pid_strafe;
  PIDController pid_turn;

  public ChassisAutoDriveVision() {
    requires(Robot.chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    NetworkTable vision = NetworkTableInstance.getDefault().getTable("Vision");
    if (Robot.grabber.getMode() == MODE.CARGO) {
      camera = vision.getSubTable("Front");
    } else {
      camera = vision.getSubTable("Rear");
    }

    pid_strafe = new PIDController(1.0, 0.0, 0.0, new StrafeSource(), new StrafeOutput());
    pid_turn = new PIDController(1.0, 0.0, 0.0, new TurnSource(), new TurnOutput());

    pid_strafe.setAbsoluteTolerance(1.0);
    pid_turn.setAbsoluteTolerance(1.0);

    pid_strafe.setSetpoint(0.0);
    pid_turn.setSetpoint(0.0);
    pid_strafe.enable();
    pid_turn.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.chassis.driveChassis(current_strafe, 0.0, current_turn);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return camera.getEntry("Lock").getBoolean(false) && !pid_strafe.onTarget() && !pid_turn.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pid_strafe.disable();
    pid_turn.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    pid_strafe.disable();
    pid_turn.disable();
  }
}
