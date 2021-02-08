/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PulleySubsystem extends SubsystemBase {
  /**
   * Creates a new PulleySubsystem.
   */
  TalonSRX pullEy = new TalonSRX(21);
  TalonSRX pulley = new TalonSRX(22);
  public PulleySubsystem() {
    
  }
  public void setPullEy(double speed){
    pullEy.set(ControlMode.PercentOutput, -speed);
    pulley.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
