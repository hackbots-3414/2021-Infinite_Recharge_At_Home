package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightAlignCommand extends CommandBase {

    private final LimelightSubsystem limelight;
    private final DrivetrainSubsystem drivetrain;
    private long startTime = System.currentTimeMillis();
    private int counter = 0;

    public LimelightAlignCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
        super();

        addRequirements(limelight);
        addRequirements(drivetrain);
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        // TODO Auto-generated method stub
        super.initialize();
        limelight.turnLEDOn();
        limelight.visionProcessor();
        Timer.delay(0.1);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("limelight y value: ", limelight.getVerticalOffset() + 20);
        SmartDashboard.putNumber("limelight x value: ", limelight.getHorizontalOffset() + 20);
        System.out.println("limelight y value: " + limelight.getVerticalOffset());
        System.out.println("////////Limelight Is On!////////");
    }

    @Override
    public boolean isFinished() {
        double tx = limelight.getHorizontalOffset();
        double ta = limelight.getTargetArea();
        double angle_tolerance = 0.5; // 0.03;
        // double ta = limelight.getTargetArea();
        if (System.currentTimeMillis() - startTime < 600 && tx == 0) {
            System.out.println("//////////////////// tx = 0 or delay reached");
            return false;
        }
        System.out.println("//////////////////// tx = " + tx);
        if (tx > -1 * angle_tolerance && tx < angle_tolerance) {
            // Increment counter for aligned to target. If we count 10 times, we must be
            // aligned
            counter++;
            drivetrain.tankDrive(0, 0);
            if (counter >= 10) {
                System.out.println("////////Exited Limelight////////");
                return true;
            } else {
                return false;
            }

        } else {
            counter = 0;
            // double kp = -0.025f;
            double heading_error = tx;
            double base = 0.25;
            double throttleFloor = 0.25;// 0.05
            double throttlePercent = 0.0;
            double angleBias = .10;// 1.5s
            // Equation to perform an inverse expontation decay of the throttle response
            double magnitude = base - 1 / Math.exp(Math.abs(heading_error + angleBias));

            throttlePercent = Math.max(Math.abs(magnitude), throttleFloor);
            throttlePercent = Math.copySign(throttlePercent, heading_error) * -1;

            /*
             * double steering_adjust = kp * heading_error;
             * System.out.println("this is steering_adjust " + steering_adjust);
             * steering_adjust =
             * Math.copySign(Math.max(steering_adjust,.20),steering_adjust);
             * System.out.println("this is steering_adjust " +steering_adjust);
             */

            // left += throttlePercent;// steering_adjust; originally subtract
            // right -= throttlePercent;// steering_adjust;
            double rotation;
            // //New alignmnet code
            if (Math.abs(tx) > 10) {
                rotation = 0.37;
            } else {
                rotation = 0.18;
            }

            if (tx > 0) {
                rotation = -rotation;
            }
            System.out.println("-------------limelight align-----------------------------");
            drivetrain.drive(0, rotation);
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Timer.delay(0.3);
        limelight.turnLEDOff();
        limelight.driverCameraVision();
    }

    public void interrupted() {
        end(false);
    }

}