/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Utilities;

public class TurnInMotion extends CommandBase {
  DrivetrainSubsystem navXDrive = null;
  double m_angle;
  public Utilities util = new Utilities();
  int m_radius;
  public double m_tolerance;
  double initialRefrenceAngle;
  boolean isFinishedend = false;
  public TurnInMotion(final DrivetrainSubsystem pidNavXDrive, double finalAngle, double tolerance, int radius) {
    navXDrive = pidNavXDrive;
    m_angle = finalAngle;
    m_tolerance = tolerance;
    m_radius = radius;
    initialRefrenceAngle = m_angle;
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (Math.abs(m_angle) > 0) {
      navXDrive.setPIDValues(util.k_PTurn, util.k_ITurn, util.k_DTurn);
      navXDrive.getController().setTolerance(0.01, 0.01);
    } else {
      navXDrive.setPIDValues(util.k_PDrive, util.k_IDrive, util.k_DDrive);
    }
    System.out.println("initialize");
    navXDrive.setSetpoint(m_angle);
    isFinishedend = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
