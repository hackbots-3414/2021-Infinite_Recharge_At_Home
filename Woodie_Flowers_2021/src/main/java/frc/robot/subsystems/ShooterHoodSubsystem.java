/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHoodSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterHoodSubsystem.
   */
  TalonSRX hoodMotor = new TalonSRX(21);
 
  public ShooterHoodSubsystem() {
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  }
  public void setHood(double targetDistance){
    System.out.println("/////////////// Entered setHood /////////////");
    if (targetDistance >= 20){ 
      while (getPotentiometer() < 800) {hoodMotor.set(ControlMode.PercentOutput, 0.4);}
      
    }else{
      while (getPotentiometer() > 15) {hoodMotor.set(ControlMode.PercentOutput, -0.4);}
    }
    hoodMotor.set(ControlMode.PercentOutput, 0);
    System.out.println("//////////// Left setHood ////////////");
  }
  public double getPotentiometer(){
    //SmartDashboard.putNumber("Shooter Hood Pot: ", hoodPot.getSelectedSensorPosition());
    // Range is between 10 and 810 on a half turn
    return hoodMotor.getSelectedSensorPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
