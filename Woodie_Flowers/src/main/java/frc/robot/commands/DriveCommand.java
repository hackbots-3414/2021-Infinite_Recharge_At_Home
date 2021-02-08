/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
  /**
   * Creates a new DriveCommand.
   */
  private final DrivetrainSubsystem drivetrainSubsystem;

  public DriveCommand(DrivetrainSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = subsystem;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // drivetrainSubsystem.setMaxOutput(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
      //drivetrainSubsystem.resetEncoders();
      drivetrainSubsystem.drive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
