package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbLeverDegrees extends Command {

    private ClimbSubsystem climb;
    private double targetDegrees;
    private boolean wait;

    public ClimbLeverDegrees(ClimbSubsystem climbSub, double degrees) {
        this(climbSub, degrees, false);
    }
    public ClimbLeverDegrees(ClimbSubsystem climbSub, double degrees, boolean waitForTarget) {
        climb = climbSub;
        targetDegrees = degrees;
        wait = waitForTarget;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setLeverDegrees(targetDegrees);
    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        
        return climb.isLeverIsAtTarget();
    }
}
