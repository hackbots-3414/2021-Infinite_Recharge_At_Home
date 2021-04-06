// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class ShooterHoodCommand extends CommandBase {
  /** Creates a new ShooterHoodCommmad. */
  ShooterHoodSubsystem shooterHood;
  double distance;
  public ShooterHoodCommand(ShooterHoodSubsystem hood, double driveDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterHood = hood;
    distance = driveDistance;
    addRequirements(shooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterHood.setHood(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
