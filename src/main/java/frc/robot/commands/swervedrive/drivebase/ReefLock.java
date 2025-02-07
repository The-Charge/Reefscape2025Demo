package frc.robot.commands.swervedrive.drivebase;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class ReefLock extends Command {
    private final SwerveSubsystem swerve;

    public ReefLock(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        List<Pose2d> tagposes = new ArrayList<>();
        for (int i = 1; i <= 22; i++) {
            if (i == 4 || i == 5 || i == 14 || i == 15 || i == 3 || i == 16)
                continue;
            tagposes.add(ApriltagConstants.TAG_POSES[i].toPose2d());
        }

        Pose2d nearestTag = swerve.getPose().nearest(tagposes);
        Rotation2d targetRotation = nearestTag.getRotation().minus(new Rotation2d(Math.PI));

        ChassisSpeeds reefLockSpeeds = swerve.getTargetSpeeds(
                0, 0,
                targetRotation.getSin(),
                targetRotation.getCos());

        swerve.drive(new Translation2d(), reefLockSpeeds.omegaRadiansPerSecond, true);

        // Optional telemetry
        SmartDashboard.putNumber("Swerve Heading", swerve.getHeading().getRadians());
        SmartDashboard.putNumber("omegaRadiansPerSecond", reefLockSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Closest Tag", nearestTag.getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}