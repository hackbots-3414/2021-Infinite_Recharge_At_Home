package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommand;

public class IRSensorSubsystem extends SubsystemBase {
    private AnalogInput irAnalog = new AnalogInput(1);
    boolean redpath = false;

    public void IRSensorSubsytem() {

    }

    // Gets raw voltage     

    public double getVoltage() {
        return irAnalog.getVoltage();
    }

    // Math to convert voltage to distance

    public double getDistance() {
        return 137.5/(getVoltage() - 1.125);

    }

public void setRedPath(boolean redpath) {
    this.redpath = redpath;

}

}

// Command to compute whether robot is looking at red or blue path

// SHARP 7Y 2Y0A710 F <-- IR Sensor Model

// Voltage per (1/cm) = 137.5d + 1.125 (Equation for IR Sensor)

// d = 137.5/(V - 1.125)