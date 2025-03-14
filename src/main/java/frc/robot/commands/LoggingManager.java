package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class LoggingManager extends Command {
    
    private final SwerveSubsystem swerve;

    private final StructPublisher<Pose2d> posePublisher;

    /**
     * Note: Doesn't require any subsystems, just needs them for querying data
     */
    public LoggingManager(SwerveSubsystem swerveSub) {
        swerve = swerveSub;

        posePublisher = NetworkTableInstance.getDefault().getStructTopic("SwervePose", Pose2d.struct).publish();
    }

    @Override
    public void execute() {
        posePublisher.set(swerve.getPose());
        System.out.println(swerve.getPose().toString());
    }
    @Override
    public void end(boolean interrupted) {
        posePublisher.close();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
