// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.LoggingManager;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TelemetryConstants;

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

    public Robot() {
        instance = this;
    }
    
    public static Robot getInstance()
    {
        return instance;
    }
    public RobotContainer getContainer() {
        return m_robotContainer;
    }
    
    /**
    * This function is run when the robot is first started up and should be used for any initialization code.
    */
    @Override
    public void robotInit() {   
        DataLogManager.start("", "", 0.125); //Use default dir and filename, All logging is dumped into either /home/lvuser/logs or a USB drive if one is connected to the robot
        DataLogManager.logNetworkTables(false); //use custom logging with LoggingManager
        DriverStation.startDataLog(DataLogManager.getLog(), true); //enable logging DS control and joystick input
        LoggingManager.logAndAutoSendValue("LimelightManager running", false);

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        m_robotContainer.getElev().resetL4Override();
        
        // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
        // immediately when disabled, but then also let it be pushed more 
        disabledTimer = new Timer();
        
        if (isSimulation())
        {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // Connect to 172.22.11.2:2011 to see reef limelight
        PortForwarder.add(2011, "limelight-reef.local", 5800);
        PortForwarder.add(2011, "limelight-reef.local", 5801);
        PortForwarder.add(2011, "limelight-reef.local", 5805);

        // Connect to 172.22.11.2:2012 to see  funnel limelight
        PortForwarder.add(2012, "limelight-funnel.local", 5800);
        PortForwarder.add(2012, "limelight-funnel.local", 5801);
        PortForwarder.add(2012, "limelight-funnel.local", 5805);
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

        LoggingManager.logAndAutoSendValue("Total Amps", m_pdp.getTotalCurrent());
        LoggingManager.logAndAutoSendValue("Battery Voltage", m_pdp.getVoltage());
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
        m_robotContainer.stopRumble();
        // m_robotContainer.scheduleLimelight();
        m_robotContainer.getLEDManager().resetEndgameStarted();
    }
    
    @Override
    public void disabledPeriodic()
    {
        if (disabledTimer.hasElapsed(SwerveConstants.WHEEL_LOCK_TIME))
        {
            m_robotContainer.setMotorBrake(false);
            disabledTimer.stop();
            disabledTimer.reset();
        }
    }
    
    /**
    * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
    */
    @Override
    public void autonomousInit()
    {
        m_robotContainer.clearTeleopDefaultCommand();
        m_robotContainer.setMotorBrake(true);
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_autonomousCommand = Commands.print("No autons in demo mode");
        
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }

        // m_robotContainer.scheduleLimelightAuton(); //use auton mode that cuts off after a certain amount of time

        m_robotContainer.displayAuto();
        m_robotContainer.getHeadSubsystem().recheckHasCoral();
        // m_robotContainer.scheduleLimelightAuton();
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
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        }
        else {
            CommandScheduler.getInstance().cancelAll();
        }
        // m_robotContainer.scheduleLimelight();
        
        m_robotContainer.setTeleopDefaultCommand();
        m_robotContainer.setMotorBrake(true);
        AutoDisplayHelper.clearAutoPath();
        m_robotContainer.getHeadSubsystem().recheckHasCoral();
        // m_robotContainer.scheduleControllerRumble();
    }
    
    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
    {
        // LoggingManager.logAndSendValue("Time Remaining", DriverStation.getMatchTime());

        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putNumber("velocity", Math.hypot(
                m_robotContainer.getSwerveSubsystem().getFieldVelocity().vxMetersPerSecond,
                m_robotContainer.getSwerveSubsystem().getFieldVelocity().vyMetersPerSecond
            ));
        }
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
