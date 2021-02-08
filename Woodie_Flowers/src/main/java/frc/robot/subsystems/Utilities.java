/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class Utilities extends SubsystemBase {
  /**
   * Add your docs here.
   */
  public Utilities() {
    // Intert a subsystem name and PID values here
    super();
    
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }
  //public double k_PTurn = 0.0129; 0.015 is a great number for course correction
  //public double k_PTurn = 0.026; for large turns
  public static final double k_PDrive = 0.0163;
  public static final double k_IDrive = 0.03;
  public static final double k_DDrive = 0.0153;
  //public double k_DTurn = 0.012;for large turns
  //public double k_DTurn = 0.0138675;
  public static final double k_PTurn = 0.0129;//0.0130;
  public static final double k_ITurn = 0.004;//0.003;
  public static final double k_DTurn = 0.0138675;//0.0138675;
  public static boolean atSetPoint = false;
  public static final double toler = 0.1;
  public static boolean abruptStop=false;
  //0.0215
  //8.6


  public static void setAtSetpoint(boolean value){
    atSetPoint = value;
  }
}
