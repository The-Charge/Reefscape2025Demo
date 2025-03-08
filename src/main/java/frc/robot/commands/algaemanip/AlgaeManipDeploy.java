package frc.robot.commands.algaemanip;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeManipConstants;
import frc.robot.subsystems.AlgaeManipSubsystem;

public class AlgaeManipDeploy extends Command {
    
    private final AlgaeManipSubsystem manip;

    public AlgaeManipDeploy(AlgaeManipSubsystem algaeManipSub) {
        manip = algaeManipSub;

        addRequirements(manip);
    }

    @Override
    public void initialize() {
        manip.setPivotDegrees(AlgaeManipConstants.activePos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
