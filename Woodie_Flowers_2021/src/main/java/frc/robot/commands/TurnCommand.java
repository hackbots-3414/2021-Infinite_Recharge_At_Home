/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
// Angle name and trun degrees also the stop all command 
public class TurnCommand extends CommandBase {
  DrivetrainSubsystem anglething;
  double angle;
  double finalAngle = 90;
  boolean end = false;

  // Change final angle of robot
  /**
   * Creates a new ExampleCommand.
   *           The subsystem used by this command.
   */
  public TurnCommand(final DrivetrainSubsystem drive) {
    anglething = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  // Set's the final angle of the robot
  @Override
  public void execute() {
    angle = anglething.getHeading();
    anglething.tankDrive(0, 0);
    // If right angle is greater then 0 then go right
    if (angle > 0) {
      if (angle < finalAngle) {
        anglething.tankDrive(0.1, -0.1);
      }
      if (angle == finalAngle) {
        anglething.tankDrive(0, 0);
        end = true;
      }
      if (angle > finalAngle) {
        anglething.tankDrive(-0.1, 0.1);
      }
    }
    // If left angle is less then 0 then go left
    if (angle < 0) {
      if (angle < finalAngle) {
        anglething.tankDrive(-0.1, 0.1);
      }
      if (angle == finalAngle) {
        anglething.tankDrive(0, 0);
        end = true;
      }
      if (angle > finalAngle) {
        anglething.tankDrive(0.1, -0.1);
      }
    }
 
    
    
  }






  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
   
    
    
   } 
}

