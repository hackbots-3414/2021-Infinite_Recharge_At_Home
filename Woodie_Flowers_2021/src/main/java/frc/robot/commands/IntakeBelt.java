// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeBelt extends ParallelCommandGroup {
  IntakeSubsystem intake;
  BeltSubsyteem belt;
  /** Creates a new IntakeBelt. */
  public IntakeBelt(IntakeSubsystem intake, BeltSubsyteem belt) {
    this.intake = intake;
    this.belt = belt;
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeCommand(intake), new BeltDotEXE(belt));
  }
}
