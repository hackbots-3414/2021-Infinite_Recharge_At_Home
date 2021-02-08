package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    double speedOfShooter;
    LimelightSubsystem limelight;
    WPI_TalonFX leftMotor = new WPI_TalonFX(12);
    WPI_TalonFX rightMotor = new WPI_TalonFX(11); //34' 16500, 13ft - 22ft 14000, 22ft - 25ft 14500, 26ft 14600, 28ft 15000/ 30ft 15500
    public static final double VELOCITY_OFFSET = 500;

    public Shooter(LimelightSubsystem limelight) {
        super();
        this.limelight = limelight;
        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor);
        init();
    }

    public double getLeftShooterVelocity() {
        return leftMotor.getSelectedSensorVelocity();
    }

    public double getRightShooterVelocity() {
        return rightMotor.getSelectedSensorVelocity();
    }

    public double getAverageShooterVelocity() {
        return (getRightShooterVelocity() + getLeftShooterVelocity()) / 2;
    }

    public void init() {
        leftMotor.config_kP(0, 0.07);//0.5
        leftMotor.config_kI(0, 0);
        leftMotor.config_kD(0, 10);//25
        leftMotor.config_kF(0, 0.051);
        leftMotor.configAllowableClosedloopError(0, 400);
        leftMotor.configVoltageCompSaturation(11);
        rightMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
        rightMotor.enableVoltageCompensation(true);
        leftMotor.config_IntegralZone(0, 600); //400*1.5    
    }

    public void shoot() {
        double limelightVerticalOffsetToRadians = Math.toRadians(limelight.getVerticalOffset());
        double distance = 74/(Math.tan(Math.toRadians(20) + limelightVerticalOffsetToRadians));
        double x = (distance/12)-2;
        SmartDashboard.putNumber("Distance from target",x); // The minus two is a fudge factor
        double speedOfShooter = (500*x)+4000;
        System.out.println("shooter velocity: " + speedOfShooter);
        leftMotor.set(ControlMode.Velocity, speedOfShooter);
        
    }

    public boolean isReadyToShoot() {
        double currentShooterVelocity = leftMotor.getSelectedSensorVelocity();
        if (currentShooterVelocity >= speedOfShooter - VELOCITY_OFFSET) {
            return true;
            // TODO return true when shooter velocity is within
            // tolerance of desired velocity
            // command.isFinished() will call this
        }
        return false;
    }

    public void stop() {
        leftMotor.set(0.0);
    }

}