// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
* described in the TimedRobot documentation. If you change the name of this class or the package after creating this
* project, you must also update the build.gradle file in the project.
*/
public class Robot extends TimedRobot {
    
    private static Robot instance;
    private Command m_autonomousCommand;
    
    private RobotContainer m_robotContainer;
    
    private Timer disabledTimer;

    private PowerDistribution m_pdp = new PowerDistribution();

    private PathPlannerPath path;
    
    public Robot() {
        try {path = PathPlannerPath.fromPathFile("path1");} catch (Exception e) {e.printStackTrace();}

        instance = this;
    }
    
    public static Robot getInstance()
    {
        return instance;
    }
    
    /**
    * This function is run when the robot is first started up and should be used for any initialization code.
    */
    @Override
    public void robotInit()
    {   
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        
        // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
        // immediately when disabled, but then also let it be pushed more 
        disabledTimer = new Timer();
        
        if (isSimulation())
        {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

            // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.

    // all pathpoints to pose2d
    // translations and roations
    List<PathPoint> pathpoints = path.getAllPathPoints();

    ArrayList<Translation2d> positions = new ArrayList<>();
    for (PathPoint point : pathpoints) {
        positions.add(point.position);
    }
    System.out.println("HI HI HI \n \n HI HI HI  \n \n \n HI H IH I\n \n \n\n \n \n \n ");
    System.out.println(pathpoints.size());
    System.out.println("\n \n \n \n \n \n \nHI HI HI \n \n HI HI HI  \n \n \n HI H IH I\n \n \n");

//Pose2d(Translation2d, ROTATION2D: pathpoints.get(pathpoints.size()-1).rotationTarget.rotation())
    ArrayList<Pose2d> poses = new ArrayList<>();
    for (PathPoint point : pathpoints) {
        if (point.rotationTarget == null) poses.add(new Pose2d(point.position, new Rotation2d()));
        else poses.add(new Pose2d(point.position, point.rotationTarget.rotation()));
    }
    Pose2d endpose = new Pose2d(pathpoints.get(pathpoints.size()-1).position, new Rotation2d());
    Pose2d startpose = new Pose2d(pathpoints.get(0).position, new Rotation2d());
    TrajectoryConfig tc = new TrajectoryConfig(3,3);
    PathConstraints pc = path.getGlobalConstraints();
    
    // change to reference pc instead of hard code
    tc.setReversed(false);
    tc.setStartVelocity(0);
    tc.setEndVelocity(0);
    tc.setKinematics(m_robotContainer.getSwerveSubsystem().getKinematics());
    // TrajectoryConstraint tCons = new TrajectoryConstraint() {
        
    // };
    // tc.addConstraints();
    // System.out.println(startpose.)
    path.getWaypoints();
    // Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(startpose, positions, endpose, tc);
    Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(poses, tc);
        // Create and push Field2d to SmartDashboard.
        Field2d m_field = new Field2d();
        m_field.getObject("traj").setTrajectory(m_trajectory);
     SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    }
    private Translation2d pointTranslation(PathPoint point) {
        return point.position;
    }
    /**
    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
    * during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic()
    {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }
    
    /**
    * This function is called once each time the robot enters Disabled mode.
    */
    @Override
    public void disabledInit()
    {
        m_robotContainer.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
    }
    
    @Override
    public void disabledPeriodic()
    {
        if (disabledTimer.hasElapsed(SwerveConstants.WHEEL_LOCK_TIME))
        {
            m_robotContainer.setMotorBrake(false);
            disabledTimer.stop();
        }
    }
    
    /**
    * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
    */
    @Override
    public void autonomousInit()
    {
        m_robotContainer.setMotorBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }
    }
    
    /**
    * This function is called periodically during autonomous.
    */
    @Override
    public void autonomousPeriodic()
    {
    }
    
    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        } else
        {
            CommandScheduler.getInstance().cancelAll();
        }
    }
    
    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
    {
        SmartDashboard.putNumber("Battery Voltage", m_pdp.getVoltage());
        SmartDashboard.putNumber("Total Amps", m_pdp.getTotalCurrent());
        SmartDashboard.putNumber("Time Remaining", DriverStation.getMatchTime());
        SmartDashboard.putNumber("velocity", Math.sqrt(Math.pow(m_robotContainer.getSwerveSubsystem().getFieldVelocity().vxMetersPerSecond, 2)
        + Math.pow(m_robotContainer.getSwerveSubsystem().getFieldVelocity().vyMetersPerSecond, 2)));

    }
    
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic()
    {
    }
    
    /**
    * This function is called once when the robot is first started up.
    */
    @Override
    public void simulationInit()
    {
    }
    
    /**
    * This function is called periodically whilst in simulation.
    */
    @Override
    public void simulationPeriodic()
    {
    }
}
