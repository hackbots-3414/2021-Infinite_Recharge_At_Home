/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.teleop.OI;

public class HookDotEXE extends CommandBase {
  /**
   * Creates a new HookDotEXE.
   */
  HookSubsystem hook;
  double speed;
  public HookDotEXE(double output,HookSubsystem hookSubsystem) {
    speed = output;
    hook = hookSubsystem;
    addRequirements(hookSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hook.setHook(speed);
    System.out.println("hook encoder vals: " + hook.getEncoder() + "Speed: " + speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hook.setHook(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
