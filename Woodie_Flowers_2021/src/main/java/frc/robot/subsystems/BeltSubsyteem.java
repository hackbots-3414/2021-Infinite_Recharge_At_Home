/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsyteem extends SubsystemBase {
  /**
   * Creates a new BeltDotEXE.
   * These are the belt motor with their respective channels
   * 
   */
    CANSparkMax topBelt = new CANSparkMax(42,MotorType.kBrushless);
    CANSparkMax midBelt = new CANSparkMax(41,MotorType.kBrushless);
    CANSparkMax lowBelt = new CANSparkMax(43,MotorType.kBrushless);
    public DigitalInput irsfront = new DigitalInput(1);
    public DigitalInput irsback = new DigitalInput(0);

    private boolean conveyorSensorFront = false;
    private boolean conveyorSensorBack = false;

  public BeltSubsyteem() {
  topBelt.setSmartCurrentLimit(30);
  midBelt.setSmartCurrentLimit(30);
  lowBelt.setSmartCurrentLimit(30);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void beltMethod(double speed) {
    topBelt.set(speed/3);// divide by 4
    midBelt.set(speed/2);
    lowBelt.set(-speed);
  }
  public int getConveyorState(){
      if (conveyorSensorFront == false && conveyorSensorBack == false){
        return 0;
      } else if (conveyorSensorBack == true && conveyorSensorFront == false){
        return 1;
      }
      return 2;
    } 
  public void setconveyorSensorfront(boolean conveyorSensorFront){
    this.conveyorSensorFront = conveyorSensorFront;
  }
  public void setconveyorSensorback(boolean conveyorSensorback){
    this.conveyorSensorBack = conveyorSensorBack;
  }
  }

