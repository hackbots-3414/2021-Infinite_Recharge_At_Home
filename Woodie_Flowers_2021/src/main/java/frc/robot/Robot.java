/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Utilities;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  Utilities values = new Utilities();
  private static final String kDefaultAuto = "Back Up 2 Meters";
  private static final String kCustomAuto = "Center Auton";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected;
  ;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_chooser.setDefaultOption("Back up 2 meters Auto", kDefaultAuto);
    m_chooser.addOption("Back up and shoot Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  boolean counter;

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    
    // schedule the autonomous command (example)
   
    
      m_autoSelected = m_chooser.getSelected();
      switch (m_autoSelected) { 
        case kCustomAuto: // Put custom auto code here break; case kDefaultAuto: default:
      // Put default auto code here break; } pidNavX.resetDistance();

      m_autonomousCommand = m_robotContainer.getAutonShoot();
      break;
        default: 
        m_autonomousCommand = m_robotContainer.getAutonNoShoot();
      }

      m_autonomousCommand = m_robotContainer.getDriveStraightCommand();
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
     
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

/*
 * @Override public void autonomousPeriodic() { boolean stooop = false;
 * 
 * if (counter == true) { counter = false; //
 * CommandScheduler.getInstance().schedule(new TurnDotEXE(pidNavX,180,3)); //
 * CommandScheduler.getInstance().schedule(new DriveDotEXE(20000,0.5));
 * pidNavX.enable(); // CommandScheduler.getInstance().schedule(forward);
 * System.out.println("worked"); pidNavX.setPreviousDistance(0);
 * CommandScheduler.getInstance().schedule(new TurnDotEXE(pidNavX, -90, 5));
 * counterV2 = false;
 * 
 * } if (forward.isFinished() == true && pidNavX.getInterupted() == true &&
 * counter == false && counterV2 == true) {
 * CommandScheduler.getInstance().cancel(forward);
 * CommandScheduler.getInstance().schedule(stay0degrees);
 * System.out.println("pidNavx schedule"); pidNavX.setInterupted(false);
 * counterV2 = false; } if (counterV2 == false && counter == false &&
 * Math.abs(pidNavX.getMeasurement()) > 85 && stooop == false) {
 * pidNavX.enable(); CommandScheduler.getInstance().schedule(forward);
 * System.out.println("drive scheduled"); counterV2 = true; stooop = true; } if
 * (stay0degrees.isFinished() == true && counterV2 == false && counter == false)
 * { CommandScheduler.getInstance().cancel(stay0degrees);
 * CommandScheduler.getInstance().schedule(forward);
 * System.out.println("drive scheduled"); counterV2 = true; }
 * 
 * }
 */