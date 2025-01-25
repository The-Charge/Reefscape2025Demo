package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class AutoDisplayHelper {

    public static void displayAutoPath(Command autoCommand, boolean isRedAlliance) {

        //The name is "InstantCommand" when Command.none() is passed
        if(autoCommand == null || autoCommand.getName().equals("InstantCommand")) {
            DriverStation.reportError("Cannot display the path of a null/empty command, use clearAutoPath instead", true);
            return;
        }

        List<PathPlannerPath> paths;
        try {
            paths = PathPlannerAuto.getPathGroupFromAutoFile(autoCommand.getName());
        }
        catch(Exception e) {
            e.printStackTrace();
            return;
        }

        //generate each trajectory and merge them into one
        Trajectory mergedTraj = null;
        for(PathPlannerPath path : paths) {
            if(isRedAlliance)
                path = path.flipPath();

            List<Pose2d> poses = getPosesFromPath(path);
            Trajectory traj = generateTrajectory(poses);

            if(mergedTraj == null) {
                mergedTraj = traj;
                continue;
            }
            mergedTraj = mergedTraj.concatenate(traj);
        }

        if(mergedTraj == null) {
            DriverStation.reportWarning("AutoDisplayHelper::displayAutoPath -> Auto has no paths, redirecting to clearAutoPath", false);
            clearAutoPath();
            return;
        }
        
        /*
         * Push field2d
         * For a reason that not even god knows you have to push a blank field then set the trajectory after in order for it to update
         */
        Field2d field = new Field2d();
        
        SmartDashboard.putData("Field", field);
        field.getObject("traj").setTrajectory(mergedTraj);
    }
    public static void clearAutoPath() {
        Field2d field = new Field2d();

        SmartDashboard.putData("Field", field);
        field.getObject("traj").setTrajectory(new Trajectory());
    }

    private static Trajectory generateTrajectory(List<Pose2d> poses) {
        TrajectoryConfig trajConfig = new TrajectoryConfig(3, 3); //values don't matter as long as they aren't zero

        return TrajectoryGenerator.generateTrajectory(poses, trajConfig);
    }
    private static Rotation2d calculatePoseRotation(PathPoint current, PathPoint next) {
        //calculate rotation from current point to next point using atan2
        double rad = Math.atan2(next.position.getY() - current.position.getY(), next.position.getX() - current.position.getX());
        return new Rotation2d(rad);
    }
    private static List<Pose2d> getPosesFromPath(PathPlannerPath path) {
        List<PathPoint> pathPoints = path.getAllPathPoints();
        List<Pose2d> poses = new ArrayList<>();

        // poses.add(path.getStartingHolonomicPose().get());

        for(int i = 0; i < pathPoints.size(); i++) {
            PathPoint pp = pathPoints.get(i);

            Rotation2d rot;
            if(pp.rotationTarget != null) //if the point already has a rotation then use it
                rot = pp.rotationTarget.rotation();
            else if(i == pathPoints.size() - 1) //edge case for if the last point doesnt have a rotation then use a default of 0
                rot = new Rotation2d();
            else //calculate the angle from the current point to the next
                rot = calculatePoseRotation(pp, pathPoints.get(i + 1));

            poses.add(new Pose2d(
                pp.position,
                rot
            ));
        }

        // poses.add(path.getPathPoses().get(path.getPathPoses().size() - 1));

        return poses;
    }
}
