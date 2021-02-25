// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
/**
 * Stop motors, reset distance, start motors, get angle, set speed, drive until distance,
 * compensate for drift/fix angle, stop motors
 */
public class DriveStraight extends CommandBase {

  private double speed;
  private double distance;
  private DrivetrainSubsystem drivetrain;
  /** Creates a new DriveStraight. */
  public DriveStraight(double speed, double distance, DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = -speed;
    this.distance = distance;
    this.drivetrain = drivetrain;  
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    drivetrain.stop();
      
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drivetrain.getAverageDistance() > distance){
      return true;
    }
    return false;
  }
}
