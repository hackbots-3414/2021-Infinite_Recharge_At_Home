/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.DriveConstants.kMaxAccelerationMetersPerSecondSquared;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.BeltDotEXE;
import frc.robot.commands.BeltShootCommand;
import frc.robot.commands.CenterAuton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveDotEXE;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HookDotEXE;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeLEDParallelCommand;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.LEDParty;
import frc.robot.commands.LEDShooterCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.commands.PulleyDotEXE;
import frc.robot.commands.ShootSequenceCommand;
import frc.robot.commands.SpinUpCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.TurnDotEXE;
import frc.robot.subsystems.BeltSubsyteem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.teleop.OI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  boolean counterV2;
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final DriveCommand m_driveCommand = new DriveCommand(m_drivetrainSubsystem);
  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  TrajectoryConfig config;
  String trajectoryJSON = "paths/center_auton_start.wpilib.json";
  Path trajectoryPath;
  Trajectory trajectory;
  private Shooter m_shooter = new Shooter(m_limelightSubsystem);
  private final LEDShooterCommand m_ledShooter = new LEDShooterCommand(m_ledSubsystem);
  TurnDotEXE stay0degrees = new TurnDotEXE(m_drivetrainSubsystem, 5, 1);
  DriveDotEXE forward = new DriveDotEXE(200000, 0.5, 6, m_drivetrainSubsystem);
  private final StopCommand m_stop = new StopCommand(m_shooter, m_drivetrainSubsystem);
  BeltSubsyteem beltDriveSubsyteem = new BeltSubsyteem();
  BeltDotEXE beltCommand = new BeltDotEXE(beltDriveSubsyteem);
  BeltShootCommand ejectBelt = new BeltShootCommand(beltDriveSubsyteem, -0.7);
  BeltShootCommand beltForward = new BeltShootCommand(beltDriveSubsyteem, 0.7); 
  HookSubsystem hookSubsystem = new HookSubsystem();
  HookDotEXE hookCommandpos = new HookDotEXE(0.4, hookSubsystem);
  HookDotEXE hookCommandneg = new HookDotEXE(-0.4, hookSubsystem);
  PulleySubsystem pulleySubsystem = new PulleySubsystem();
  PulleyDotEXE pullyCommandpos = new PulleyDotEXE(0.9, pulleySubsystem);
  Command autonShoot;
  Command autonNoShoot;
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // private final BeltSubsyteem m_belt = new BeltSubsyteem();
  // BeltDotEXE beltCommand = new BeltDotEXE(m_belt, m_intake);

  // private final DrivetrainSubsystem m_drivetrainSubsystem = new
  // DrivetrainSubsystem();
  // private final DriveCommand m_driveCommand = new
  // DriveCommand(m_drivetrainSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSubsystem, m_driveCommand);
    CommandScheduler.getInstance().setDefaultCommand(beltDriveSubsyteem, beltCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_ledSubsystem,
        new LEDDefaultCommand(m_ledSubsystem, beltDriveSubsyteem));
    configureButtonBindings();
    m_limelightSubsystem.driverCameraVision();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    autonShoot = GetCenter();
    autonNoShoot = getAutonomousCommand();
    hookSubsystem.resetEncoder();
  }
  public Command getAutonShoot() {
    return autonShoot;
  }
  public Command getAutonNoShoot(){
    return autonNoShoot;
  }
  public Command getAutonomousCommand() {

    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 5);
    config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint).setReversed(true);
    // An example trajectory to follow. All units in meters.
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
      // TODO Auto-generated catch block
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      ex.printStackTrace();
    }
   /* Trajectory backwardsCurve = TrajectoryGenerator.generateTrajectory(
        new Pose2d(6.724, -2.166, new Rotation2d(Math.PI)), List.of(),
        new Pose2d(2.732, -2.166, new Rotation2d(Math.PI)), config); */
    Trajectory backwards = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(),
        new Pose2d(-2, 0, new Rotation2d(0)), config);

    /*Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),

        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);*/
    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
          double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };

    RamseteCommand ramseteCommand = new RamseteCommand(backwards, m_drivetrainSubsystem::getPose, disabledRamsete,
        // new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_drivetrainSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrainSubsystem::tankDriveVolts, m_drivetrainSubsystem

    );
    return ramseteCommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0));

  }
  public Command shootDistance() {

    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 5);
    config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint).setReversed(true);
    // An example trajectory to follow. All units in meters.
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
      // TODO Auto-generated catch block
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      ex.printStackTrace();
    }
   /* Trajectory backwardsCurve = TrajectoryGenerator.generateTrajectory(
        new Pose2d(6.724, -2.166, new Rotation2d(Math.PI)), List.of(),
        new Pose2d(2.732, -2.166, new Rotation2d(Math.PI)), config); */
    Trajectory backwards = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(),
        new Pose2d(-2 /*-4.5*/, 0, new Rotation2d(0)), config);

    /*Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),

        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);*/
    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
          double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };

    RamseteCommand ramseteCommand = new RamseteCommand(backwards, m_drivetrainSubsystem::getPose, disabledRamsete,
        // new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_drivetrainSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrainSubsystem::tankDriveVolts, m_drivetrainSubsystem

    );
    return ramseteCommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0));

  }
  public Command GetCenter(){
    AlignAndShootCommand autonShoot = new AlignAndShootCommand(m_limelightSubsystem, m_drivetrainSubsystem, m_shooter);
   CenterAuton centerStart = new CenterAuton(shootDistance(), autonShoot, new BeltShootCommand(beltDriveSubsyteem, 0.5), m_stop);
    return centerStart;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    System.out.println("---------------inside configureButtonBindings()");

    whileHeldDriverPadButton(new LimelightAlignCommand(m_limelightSubsystem, m_drivetrainSubsystem),
        OI.Y_BTN_LIMELIGHTALIGN);
    // whileHeldOperatorPadButton(new AlignAndShootCommand(m_limelightSubsystem,
    // m_drivetrainSubsystem, m_shooter), OI.X_BTN_ALIGNANDSHOOT);
    whileHeldDriverPadButton(
        new ShootSequenceCommand(beltDriveSubsyteem, m_drivetrainSubsystem, m_shooter, m_ledSubsystem, m_intake, m_limelightSubsystem),
        OI.B_BTN_SHOOTSEQUENCE);
    //whileHeldOperatorPadButton(new IntakeLEDParallelCommand(m_intake, m_ledSubsystem), OI.B_BTN_INTAKE);
    //whileHeldOperatorPadButton(new IntakeCommand(m_intake), OI.B_BTN_INTAKE);
    whileHeldOperatorPadButton(new LEDParty(m_ledSubsystem), OI.B_BTN_INTAKE);
    whileHeldOperatorPadButton(hookCommandpos, OI.LB_BTN_HOOK_POSITIVE);
    whileHeldOperatorPadButton(hookCommandneg, OI.RB_BTN_HOOK_NEGATIVE);
    whileHeldOperatorPadButton(pullyCommandpos, OI.X_BTN_PULLY);
    whileHeldOperatorPadButton(ejectBelt, 8);
    whileHeldOperatorPadButton(beltForward, 7);
  }

  public void whileHeldOperatorPadButton(final Command command, final int buttonNumber) {
    final JoystickButton joystickButton = new JoystickButton(OI.getOperatorPad(), buttonNumber);
    joystickButton.whileHeld(command);
  }

  public void whenPressedOperatorPadButton(final Command command, final int buttonNumber) {
    final JoystickButton joystickButton = new JoystickButton(OI.getOperatorPad(), buttonNumber);
    joystickButton.whenPressed(command);
  }


  public void whileHeldDriverPadButton(final Command command, final int buttonNumber) {
    final JoystickButton joystickButton = new JoystickButton(OI.getDrivePad(), buttonNumber);
    joystickButton.whileHeld(command);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
