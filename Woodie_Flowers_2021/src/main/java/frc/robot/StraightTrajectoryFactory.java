// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.BeltDotEXE;
import frc.robot.commands.IntakeBelt;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.commands.ShootSequenceCommand;
import frc.robot.commands.SlowShootSequenceCommand;
import frc.robot.commands.StopCommand;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class StraightTrajectoryFactory {
    public static Trajectory createStraightTrajectory(double distance) {
        boolean reversed = distance < 0 ? true : false;
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, 5);
        TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(reversed);
        // An example trajectory to follow. All units in meters.

        Trajectory backwards = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(),
                new Pose2d(distance, 0, new Rotation2d(0)), config);

        return backwards;
    }

    public static Command createDriveDistance(double distance, DrivetrainSubsystem drivetrainSubsystem) {
        RamseteController disabledRamsete = new RamseteController() {
            @Override
            public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                    double angularVelocityRefRadiansPerSecond) {
                return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
            }
        };

        RamseteCommand ramseteCommand = new RamseteCommand(StraightTrajectoryFactory.createStraightTrajectory(distance),
                drivetrainSubsystem::getPose, disabledRamsete,

                // new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, drivetrainSubsystem::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrainSubsystem::tankDriveVolts, drivetrainSubsystem);
        ramseteCommand.addRequirements(drivetrainSubsystem);
        return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));

    }

    public static Command createInterstellarACommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelight, Shooter shooter, IntakeSubsystem intake,
    BeltSubsyteem belt, LEDSubsystem ledSubsystem) {
    
        SequentialCommandGroup result = new SequentialCommandGroup();
        //Red zone
        result.addCommands(createDriveDistance(2, drivetrainSubsystem));
        result.addCommands(new ShootSequenceCommand(belt, drivetrainSubsystem, shooter, ledSubsystem, intake, limelight).withTimeout(3));
        result.addCommands(createDriveDistance(-1.5, drivetrainSubsystem));
        result.addCommands(new IntakeBelt(intake, belt).withTimeout(5));
        
        //Blue zone
        result.addCommands(createDriveDistance(3.5, drivetrainSubsystem));
        result.addCommands(new ShootSequenceCommand(belt, drivetrainSubsystem, shooter, ledSubsystem, intake, limelight).withTimeout(3));
        result.addCommands(createDriveDistance(-3.5, drivetrainSubsystem));
        result.addCommands(new IntakeBelt(intake, belt).withTimeout(5));
        //Yellow zone
        result.addCommands(new LimelightAlignCommand(limelight, drivetrainSubsystem));
        result.addCommands(createDriveDistance(5.1, drivetrainSubsystem));
        result.addCommands(new SlowShootSequenceCommand(belt, drivetrainSubsystem, shooter, ledSubsystem, intake, limelight).withTimeout(3));
        result.addCommands(createDriveDistance(-5.1, drivetrainSubsystem));
        result.addCommands(new IntakeBelt(intake, belt).withTimeout(5));
        //Green zone
        result.addCommands(new LimelightAlignCommand(limelight, drivetrainSubsystem));
        result.addCommands(createDriveDistance(6.8, drivetrainSubsystem));
        result.addCommands(new SlowShootSequenceCommand(belt, drivetrainSubsystem, shooter, ledSubsystem, intake, limelight).withTimeout(3));
        result.addCommands(createDriveDistance(-6.8, drivetrainSubsystem));
        result.addCommands(new IntakeBelt(intake, belt).withTimeout(5));
        //Last 3 balls from red zone
        result.addCommands(createDriveDistance(1.5, drivetrainSubsystem));
        result.addCommands(new ShootSequenceCommand(belt, drivetrainSubsystem, shooter, ledSubsystem, intake, limelight).withTimeout(3));
        result.addCommands(createDriveDistance(-1.5, drivetrainSubsystem));

        
        return result;
    }

}
