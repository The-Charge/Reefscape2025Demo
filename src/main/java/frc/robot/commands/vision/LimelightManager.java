package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LimelightManager extends Command {
    private final SwerveSubsystem swerve;
    private VisionSubsystem reefLimelight;
    private VisionSubsystem funnelLimelight;

    public LimelightManager(SwerveSubsystem swerve, VisionSubsystem reefLimelight, VisionSubsystem funnnelLimelight) {
        this.swerve = swerve;
        this.reefLimelight = reefLimelight;
        this.funnelLimelight = funnnelLimelight;
    }

    @Override
    public void initialize() {
        System.out.println("LimelightManager Initialize");
    }
    
    @Override
    public void execute() {
        double yaw = swerve.getHeading().getDegrees();
        double yawRate = swerve.getSwerveDrive().getGyro().getYawAngularVelocity().in(DegreesPerSecond);

        Pose2d reefPose = reefLimelight.getEstimatedPose(yaw, yawRate);
        Pose2d funnelPose = funnelLimelight.getEstimatedPose(yaw, yawRate);

        double reefTime = reefLimelight.getPoseTimestamp();
        double funnelTime = funnelLimelight.getPoseTimestamp();

        double reefAmbig = reefLimelight.getAmbiguity();
        double funnelAmbig = funnelLimelight.getAmbiguity();

        int reefCount = reefLimelight.getTagCount();
        int funnelCount = funnelLimelight.getTagCount();

        SmartDashboard.putNumber("reef ambig", reefAmbig);
        SmartDashboard.putNumber("funnel ambig", funnelAmbig);
        
        SmartDashboard.putBoolean("reef estimated", false);
        SmartDashboard.putBoolean("funnel estimated", false);

        boolean reefEstim = (reefPose != null && reefAmbig < 0.2 || reefCount > 1);
        boolean funnelEstim = (reefPose != null && reefAmbig < 0.2 || funnelCount > 1);

        if (reefEstim && funnelEstim) {
            if (reefCount > funnelCount) {
                SmartDashboard.putBoolean("reef estimated", true);
                swerve.addVisionReading(reefPose, reefTime);
            } else if (reefCount < funnelCount) {
                SmartDashboard.putBoolean("funnel estimated", true);
                swerve.addVisionReading(funnelPose, funnelTime);
            } else if (reefAmbig < funnelAmbig) {
                SmartDashboard.putBoolean("reef estimated", true);
                swerve.addVisionReading(reefPose, reefTime);
            } else {
                SmartDashboard.putBoolean("funnel estimated", true);
                swerve.addVisionReading(funnelPose, funnelTime);
            }
        } else if (reefEstim) {
            SmartDashboard.putBoolean("reef estimated", true);
            swerve.addVisionReading(reefPose, reefTime);
        } else if (funnelEstim) {
            SmartDashboard.putBoolean("funnel estimated", true);
            swerve.addVisionReading(funnelPose, funnelTime);
        }

        // Rotation2d swerveRot = swerve.getHeading();

        // if (reefPose == null && funnelPose == null) {
        //     return;
        // } else if (reefPose == null) {
        //     avgPose = funnelPose;
        // } else if (funnelPose == null) {
        //     avgPose = reefPose;
        // } else {
        //     double avgX = (reefPose.getX() + funnelPose.getX()) / 2;
        //     double avgY = (reefPose.getY() + funnelPose.getY()) / 2;
        //     // Rotation2d avgRot = reefPose.getRotation().plus(funnelPose.getRotation()).times(0.5);

        //     avgPose = new Pose2d(avgX, avgY, swerveRot);
        // }    

        // if (Double.isNaN(reefTime) && Double.isNaN(funnelTime)) {
        //     System.err.println("Limelight could not get timestamp.");
        // } else if (Double.isNaN(reefTime)) {
        //     avgTime = funnelTime;
        // } else if (Double.isNaN(funnelTime)) {
        //     avgTime = reefTime;
        // } else {
        //     avgTime = (reefTime + funnelTime) / 2;
        // }

        // swerve.addVisionReading(avgPose, avgTime);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
