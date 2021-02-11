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
    }
    @Override
    public void execute() {
        limelight.turnLEDOn();
        limelight.visionProcessor();
        Timer.delay(0.3); 
        SmartDashboard.putNumber("limelight y value: ", limelight.getVerticalOffset() + 20);
        System.out.println("limelight y value: " + limelight.getVerticalOffset());
        System.out.println("////////Limelight Is On!////////");
    }
    @Override
    public boolean isFinished() {
        double tx = limelight.getHorizontalOffset();
        double ta = limelight.getTargetArea();
        double angle_tolerance = 0.9; //0.03;
        // double ta = limelight.getTargetArea();
        if (System.currentTimeMillis() - startTime < 300 && tx == 0){
            return false; 
        }
        if (tx > -1 * angle_tolerance && tx < angle_tolerance) {
        System.out.println("////////Exited Limelight////////");
            drivetrain.tankDrive(0, 0);
            return true;
        } else {

            // double kp = -0.025f;
            double heading_error = tx;
            double base = 0.25;
            double throttleFloor = 0.18;//0.05
            double throttlePercent = 0.0;
            double angleBias = .10;//1.5s
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

            double left = 0.0;
            left += throttlePercent;// steering_adjust; originally subtract
            double right = 0.0;
            right -= throttlePercent;// steering_adjust;

            // //New alignmnet code
            // if (tx > 0) {
            // //Pivot Left
            // left = 0.2;
            // right = -0.2;
            // } else {
            // //Pivot right
            // left = -0.2;
            // right = 0.2;
            // }
            drivetrain.tankDrive(left, right);
            return false;
        }

    }
    @Override
    public void end(boolean interrupted) {
       //Timer.delay(0.3);
        limelight.turnLEDOff();
       limelight.driverCameraVision();
    }

    public void interrupted() {
        end(false);
    }

}