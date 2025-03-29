package frc.robot.commands.elev;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.ElevSubsystem.Level;

public class AdjustL4Override extends Command {
    
    private final ElevSubsystem elev;
    private final double delta;

    public AdjustL4Override(ElevSubsystem elevSub, double offset) {
        elev = elevSub;
        delta = offset;

        addRequirements(elev);
    }

    @Override
    public void initialize() {
        elev.setL4Override(elev.getL4Override() + delta);
        elev.setTargetPositionLevel(Level.LVL4);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
