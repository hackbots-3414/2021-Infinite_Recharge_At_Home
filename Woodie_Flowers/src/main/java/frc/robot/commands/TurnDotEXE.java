/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Utilities;;

public class TurnDotEXE extends CommandBase {
  DrivetrainSubsystem navXDrive = null;
  double m_angle;
  public Utilities util = new Utilities();
  public double m_tolerance;
  double initialRefrenceAngle;
  boolean isFinishedend = false;

  public TurnDotEXE(final DrivetrainSubsystem pidNavXDrive, double angularBruhMoment, double m_tolerancei) {

    navXDrive = pidNavXDrive;
    m_angle = angularBruhMoment;
    m_tolerance = m_tolerancei;
    initialRefrenceAngle = m_angle;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
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
  public void execute() {
    // System.out.println("execute");
    // System.out.println("setSetpoint "+ navXDrive.getController().getSetpoint());
    // System.out.println("Ross's will to live");
    if (Math.abs(navXDrive.getController().getPositionError()) < m_tolerance) {
      // System.out.println("isFinished: " + true);
      System.out.println("Misa finished");
      navXDrive.setInterupted(false);
      util.setAtSetpoint(true);
      isFinishedend = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // System.out.println("isFinished: " + navXDrive.atSetPoint());
    // System.out.println("tolerance: "+ util.toler);
    // System.out.println("tolerance: "+ m_tolerance);
    return isFinishedend;
  }

  // Called once after isFinished returns true

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

}
