package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LoggingManager extends Command {
    
    private final SwerveSubsystem swerve;
    private final VisionSubsystem reef;
    private final VisionSubsystem funnel;

    private final StructPublisher<Pose2d> posePublisher;
    private final StructArrayPublisher<Pose3d> reefTagPublisher;
    private final StructArrayPublisher<Pose3d> funnelTagPublisher;

    /**
     * Note: Doesn't require any subsystems, just needs them for querying data
     */
    public LoggingManager(SwerveSubsystem swerveSub, VisionSubsystem reefSub, VisionSubsystem funnelSub) {
        swerve = swerveSub;
        reef = reefSub;
        funnel = funnelSub;

        //I never close these because they should only be closed when the robot is turned off, and any memory leaks will be deleted by RAM shutting down
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("AvScope/SwervePose", Pose2d.struct).publish();
        reefTagPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("AvScope/ReefTags", Pose3d.struct).publish();
        funnelTagPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("AvScope/FunnelTags", Pose3d.struct).publish();
    }

    @Override
    public void execute() {
        posePublisher.set(swerve.getPose());
        reefTagPublisher.set(reef.getTagPose3ds());
        funnelTagPublisher.set(funnel.getTagPose3ds());
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
