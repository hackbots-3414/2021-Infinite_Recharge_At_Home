/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;

public class StopCommand extends CommandBase {
  private Shooter shooter;
  private DrivetrainSubsystem drivetrain;

  public StopCommand(Shooter shooter, DrivetrainSubsystem drivetrain) {
    super();
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    super.execute();
    // limelight.stop();
    shooter.stop();
    drivetrain.stop();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(final boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  public void interrupted() {
  }
}
