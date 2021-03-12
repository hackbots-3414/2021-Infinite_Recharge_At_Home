// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IRSensorSubsystem;

public class PathDetectorCommand extends CommandBase {
  private IRSensorSubsystem irsubsystem;

  /** Creates a new PathDetectorCommand. */

  public PathDetectorCommand(IRSensorSubsystem irsubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.

    this.irsubsystem = irsubsystem;
    addRequirements(irsubsystem);
    
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = irsubsystem.getDistance();
    boolean redpath = false;
    if(distance >= 121.92 && distance <= 182.88) {
      redpath = true;
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
