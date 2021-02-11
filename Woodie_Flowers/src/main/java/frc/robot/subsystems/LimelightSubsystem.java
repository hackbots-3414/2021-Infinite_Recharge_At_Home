package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {
        super();
    }

    public double getVerticalOffset() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        return ty;
    }

    public double getHorizontalOffset() {
        double tx = (double) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getNumber(0);
        System.out.println("-------------------------tx value: " + tx);
        return tx;
    }

    public double getTargetArea() {
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        return ta;
    }

    public double getSkew(){
        double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
        return ts;
    }

    public double getHorizonalSideLength() {
        double thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
        return thor;
    }

    public double getVerticalSideLength() {
        double tver = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tver").getDouble(0);
        return tver;
    }

    public double getLongestSide() {
        double tlong = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
        return tlong;
    }

    public double getShortestSide() {
        double tshort = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
        return tshort;
    }

    public void turnLEDOn () {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void turnLEDOff() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void visionProcessor() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }

    public void driverCameraVision() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }

}
