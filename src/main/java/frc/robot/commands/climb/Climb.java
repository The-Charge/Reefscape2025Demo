package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends Command {
    
    private ClimbSubsystem climb;

    public Climb(ClimbSubsystem climbSub) {
        climb = climbSub;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.vbus(ClimbConstants.maxVBus); //relies on soft limits to stop from going too far
    }
    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
