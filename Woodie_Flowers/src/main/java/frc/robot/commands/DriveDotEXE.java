/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Utilities;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveDotEXE extends CommandBase {
  /**
   * Creates a new DriveDotEXE.
   */
  int distan;
  public double okBoomer;
  DrivetrainSubsystem drivetrain;
  Utilities values;
  double m_tolerance;

  public DriveDotEXE(int distance, double speed, double tolerance, DrivetrainSubsystem driver) {
    okBoomer = speed;
    drivetrain = driver;
    distan = distance;
    m_tolerance = tolerance;

  }

  // Returns true when the command should end.
  @Override
  public void initialize() {
    values.abruptStop = false;
    // CommandScheduler.getInstance().schedule( new TurnDotEXE(drivetrain, 0, 0 ));
  }

  @Override
  public void execute() {

    drivetrain.drive(okBoomer, 0);

    // System.out.println("right encoder values : " +
    // drivetrain.getRightDistance());
    // System.out.println("left encoder values : " + -drivetrain.getLeftDistance());
    // CommandScheduler.getInstance().schedule( new TurnDotEXE(drivetrain, 0, 0 ));
    // 10,000 = 17 inches
    // 20,000 = 16 inches
  }

  @Override
  public boolean isFinished() {
    if (drivetrain.getMeasurement() > m_tolerance) {
      drivetrain.setInterupted(true);
      drivetrain.setPreviousDistance(drivetrain.getRightDistance() + drivetrain.getPreviousDistance());
      return true;
    }
    if (drivetrain.getMeasurement() < -m_tolerance) {
      drivetrain.setInterupted(true);
      drivetrain.setPreviousDistance(drivetrain.getRightDistance() + drivetrain.getPreviousDistance());
      return true;
    }

    if (-drivetrain.getLeftDistance() > distan && drivetrain.getRightDistance() > distan && distan > 0) {
      drivetrain.disable();
      drivetrain.drive(0.0, 0.0);
      System.out.println("isFinished = true");
      values.atSetPoint = true;
      return true;
    }
    if (-drivetrain.getLeftDistance() < distan && drivetrain.getRightDistance() < distan && distan < 0) {
      drivetrain.disable();
      drivetrain.drive(0.0, 0.0);
      System.out.println("isFinished = true");
      values.atSetPoint = true;
      return true;
    } else {
      return false;
    }
  }
}
