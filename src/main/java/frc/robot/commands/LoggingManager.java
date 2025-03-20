package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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

        //I never close these because they should only be closed when the robot is turned off, and any memory leaks will be deleted by RAM shutting down
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("AvScope/SwervePose", Pose2d.struct).publish();
    }

    @Override
    public void execute() {
        posePublisher.set(swerve.getPose());
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
