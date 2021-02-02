/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Drive extends ParallelCommandGroup {
  /**
   * Add your docs here.
   */
  int dist;
  double speeder;
  DrivetrainSubsystem lumbago;
  double m_tolerance;

  public Drive(int distance, double speed, DrivetrainSubsystem drivetrain, double tolerance) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    dist = distance;
    speeder = speed;
    lumbago = drivetrain;
    lumbago.setDriveActive(true);
    m_tolerance = tolerance;
    addCommands(new DriveDotEXE(dist, speeder, m_tolerance, lumbago), new TurnDotEXE(lumbago, 0, 0));
    parallel(new DriveDotEXE(dist, speeder, m_tolerance, lumbago), new TurnDotEXE(lumbago, 0, 0));
  }
}
