package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

//import edu.wpi.first.wpilibj.examples.frisbeebot.Constants.ShooterConstants;

public class ShooterSubsystem extends PIDSubsystem {

    WPI_TalonFX leftMotor = new WPI_TalonFX(11);
    WPI_TalonFX rightMotor = new WPI_TalonFX(12);
    private final SimpleMotorFeedforward m_shooterFeedForward = new SimpleMotorFeedforward(0, 0);

    public ShooterSubsystem() {

        super(new PIDController(0.8, 0, 0));
        getController().setTolerance(Double.POSITIVE_INFINITY, 180);
        leftMotor.setInverted(true);
        rightMotor.follow(leftMotor);
        // controller.enableContinuousInput(-11000, 11000);
        // TODO Auto-generated constructor stub
    }

    @Override
    protected void useOutput(final double output, final double setpoint) {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("averageShooterVelocity", output);
        // leftMotor.set(output);
        // rightMotor.set(-output);
        System.out.println("this is the output: " + output);
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return getAverageShooterVelocity();
    }

    public void setShooterVelocity(double setpoint) {
        setSetpoint(setpoint);
    }

    public double getLeftShooterVelocity() {
        return leftMotor.getSelectedSensorVelocity();
    }

    public double getRightShooterVelocity() {
        return -rightMotor.getSelectedSensorVelocity();
    }

    public double getAverageShooterVelocity() {
        double averageShooterVelocity = getLeftShooterVelocity() + getRightShooterVelocity() / 2;
        return averageShooterVelocity;
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public double getSetpoint() {
        return getController().getSetpoint();
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

}