/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ColorSystem;
import frc.robot.subsystems.LEDSubsystem;

public class LEDTransportFullYellow extends CommandBase {
  private LEDSubsystem ledSubsystem;

  /**
   * Creates a new LEDDefaultCommand.
   */
  public LEDTransportFullYellow(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Led.DefaultCommand.Execute()");
    ledSubsystem.switchOnLEDs(ColorSystem.COLOR_YELLOW, ColorSystem.COLOR_PATTERN_SOLID);
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
