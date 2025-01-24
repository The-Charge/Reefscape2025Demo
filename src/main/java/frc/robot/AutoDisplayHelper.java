package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class AutoDisplayHelper {

    public static void displayAutoPath(Command autoCommand) throws IOException, org.json.simple.parser.ParseException {
        // SmartDashboard.clearPersistent("Field");
        // Field2d field = new Field2d();
        Field2d field;
        // if(SmartDashboard.containsKey("Field")) {
        //     field = (Field2d) SmartDashboard.getData("Field");
        // }
        // else {
            field = new Field2d();
        // }

        //The name is "InstantCommand" when Command.none() is passed
        if(autoCommand == null || autoCommand.getName().equals("InstantCommand")) {
            System.out.println("No auto");
            //push field2d with no trajectory
            // field.getObject("traj").setTrajectory(new Trajectory());
            List<Pose2d> emptyPoses = new ArrayList<>();
            emptyPoses.add(new Pose2d(0, 0, new Rotation2d()));
            emptyPoses.add(new Pose2d(6, 9, new Rotation2d()));
            field.getObject("traj").setTrajectory(
                TrajectoryGenerator.generateTrajectory(
                    emptyPoses,
                    new TrajectoryConfig(3, 3)
                )
            );
            SmartDashboard.putData("Field", field);
            return;
        }

        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoCommand.getName());

        List<Pose2d> poses = mergePaths(paths);

        Trajectory traj = generateTrajectory(poses);
        
        //push Field2d to SmartDashboard
        field.getObject("traj").setTrajectory(traj);
        SmartDashboard.putData("Field", field);
    }

    private static List<Pose2d> mergePaths(List<PathPlannerPath> paths) {
        List<Pose2d> poses = new ArrayList<>();

        //merge all of the Pose2ds from each path into one list
        for(PathPlannerPath path : paths) {
            poses.addAll(path.getPathPoses());
        }

        return poses;
    }
    private static Trajectory generateTrajectory(List<Pose2d> poses) {
        TrajectoryConfig trajConfig = new TrajectoryConfig(3, 3); //values don't matter as long as they aren't zero

        return TrajectoryGenerator.generateTrajectory(poses, trajConfig);
    }
}
