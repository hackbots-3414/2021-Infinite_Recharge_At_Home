package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax intakeMotor = new CANSparkMax(31, MotorType.kBrushless);
    Solenoid intake = new Solenoid(0);

    public IntakeSubsystem() {
        intakeMotor.setSmartCurrentLimit(40);
    }

    public void goDown() {
        //System.out.println("intake go down");
        intake.set(true);
    }

    public void goUp() {
        intake.set(false);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(-speed);
    }

    public void stop() {
        intakeMotor.set(0.0);
    }
}