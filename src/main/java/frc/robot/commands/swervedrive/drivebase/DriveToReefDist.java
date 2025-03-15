package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToReefDist extends Command {
    
    private final SwerveSubsystem swerve;
    private final HeadSubsystem head;
    private Timer timeout;
    private PIDController pid;

    public DriveToReefDist(SwerveSubsystem swerveSub, HeadSubsystem headSub) {
        swerve = swerveSub;
        head = headSub;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        timeout.start();

        pid = new PIDController(SwerveConstants.alignPID.kP, SwerveConstants.alignPID.kI, SwerveConstants.alignPID.kD);
        pid.setSetpoint(SwerveConstants.reefAcceptableDist);
    }
    @Override
    public void execute() {
        double out = -pid.calculate(head.getFrontDistance());
        SmartDashboard.putNumber("Swerve AlignToReefDist pid", out);
        swerve.drive(new Translation2d(MathUtil.clamp(out  * SwerveConstants.reefAlignSpeed, -0.3, 0.3), 0), 0, false);
    }
    @Override
    public void end(boolean interrupted) {
        swerve.drive(Translation2d.kZero, 0, false);
    }

    @Override
    public boolean isFinished() {
        return head.getFrontDistance() <= SwerveConstants.reefAcceptableDist;
    }
}
