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

public class HookSubsystem extends SubsystemBase {
  /**
   * Creates a new HookSubsystem.
   */
  TalonSRX captainHook = new TalonSRX(51);
 
  public HookSubsystem() {
    captainHook.configReverseSoftLimitThreshold(-17000);
    captainHook.configReverseSoftLimitEnable(true);
  }
  public void setHook(double speed){
    captainHook.set(ControlMode.PercentOutput, speed);
  }
  public int getEncoder(){
    return captainHook.getSelectedSensorPosition();
  }
  public void setEncoder(int value){
    captainHook.setSelectedSensorPosition(value);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoder() {
    captainHook.setSelectedSensorPosition(0);
  }
}
