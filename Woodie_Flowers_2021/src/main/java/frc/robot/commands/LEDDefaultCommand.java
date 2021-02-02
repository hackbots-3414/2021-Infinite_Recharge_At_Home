/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ColorSystem;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.subsystems.LEDSubsystem;

public class LEDDefaultCommand extends CommandBase {
  private LEDSubsystem ledSubsystem;
  private BeltSubsyteem beltSubsyteem;

  public int getConveyorState() {
    // 0 = empty (purple)
    // 1 = some (flashing)
    // 2 = full (solid)
    return beltSubsyteem.getConveyorState();
  }

  public int getLimelightState() {
    // 0 = no target found (red)
    // 1 = target found (yellow)
    // 2 = target alligned (green)
    return 0;
  }

  /**
   * Creates a new LEDDefaultCommand.
   */
  public LEDDefaultCommand(LEDSubsystem ledSubsystem, BeltSubsyteem beltSubsyteem) {
    this.ledSubsystem = ledSubsystem;
    this.beltSubsyteem = beltSubsyteem;

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
    //ledSubsystem.switchOnLEDs(ColorSystem.COLOR_PURPLE, ColorSystem.COLOR_PATTERN_PULSE);

    int conveyorState = getConveyorState();
    int limelightState = getLimelightState();

    double matchTime = getMatchTimeRemaining();

    if (conveyorState == 0 && limelightState == 0) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_PURPLE, ColorSystem.COLOR_PATTERN_PULSE);
    } else if (conveyorState == 0 && limelightState == 1) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_PURPLE, ColorSystem.COLOR_PATTERN_PULSE);
    } else if (conveyorState == 0 && limelightState == 2) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_PURPLE, ColorSystem.COLOR_PATTERN_PULSE);
    } else if (conveyorState == 1 && limelightState == 0) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_RED, ColorSystem.COLOR_PATTERN_PULSE);
    } else if (conveyorState == 1 && limelightState == 1) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_YELLOW, ColorSystem.COLOR_PATTERN_PULSE);
    } else if (conveyorState == 1 && limelightState == 2) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_GREEN, ColorSystem.COLOR_PATTERN_PULSE);
    } else if (conveyorState == 2 && limelightState == 0) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_RED, ColorSystem.COLOR_PATTERN_SOLID);
    } else if (conveyorState == 2 && limelightState == 1) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_YELLOW, ColorSystem.COLOR_PATTERN_SOLID);
    } else if (conveyorState == 2 && limelightState == 2) {
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_GREEN, ColorSystem.COLOR_PATTERN_SOLID);
    }
    int intPart = (int) matchTime;
    //System.out.println("intPart: " + intPart + " from matchTime: " + matchTime);
    
    if (!DriverStation.getInstance().isAutonomous() && matchTime <= 10 && intPart % 2 == 1) {
      System.out.println("yellow");
        ledSubsystem.switchOnLEDs(ColorSystem.COLOR_YELLOW, ColorSystem.COLOR_PATTERN_SOLID);
    } else if (!DriverStation.getInstance().isAutonomous() && matchTime <= 16 && matchTime >= 10 && intPart % 3 == 0) {
      System.out.println("blue");
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_LIGHT_BLUE, ColorSystem.COLOR_PATTERN_SOLID);
    } else if (!DriverStation.getInstance().isAutonomous() && matchTime <= 30 && matchTime >= 16 && intPart % 4 == 1) {
      System.out.println("red");
      ledSubsystem.switchOnLEDs(ColorSystem.COLOR_RED, ColorSystem.COLOR_PATTERN_SOLID);
    }
   // ledSubsystem.switchOnLEDs(ColorSystem.COLOR_LIGHT_BLUE, ColorSystem.COLOR_PATTERN_PULSE);
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

  public double getMatchTimeRemaining() {
    // return 150 - DriverStation.getInstance().getMatchTime();
    return DriverStation.getInstance().getMatchTime();
  }
}
