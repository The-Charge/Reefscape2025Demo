package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HeadSubsystem;

public class IntakeRumble extends Command {
    
    private final HeadSubsystem head;
    private final CommandXboxController driver1, driver2;

    private boolean hasCoralLast;
    private Timer rumbleTimer;

    private final double rumbleTime = 2; //secs

    public IntakeRumble(HeadSubsystem headSub, CommandXboxController driver1, CommandXboxController driver2) {
        head = headSub;
        this.driver1 = driver1;
        this.driver2 = driver2;
    }

    @Override
    public void initialize() {
        hasCoralLast = false;
        rumbleTimer = new Timer();
    }
    @Override
    public void execute() {
        boolean hasCoral = head.getHasCoral();

        if(hasCoral && !hasCoralLast) {
            driver1.setRumble(RumbleType.kBothRumble, 1);
            driver2.setRumble(RumbleType.kBothRumble, 1);

            rumbleTimer.reset();
            rumbleTimer.start();
        }
        else if(rumbleTimer.hasElapsed(rumbleTime)) {
            driver1.setRumble(RumbleType.kBothRumble, 0);
            driver2.setRumble(RumbleType.kBothRumble, 0);

            rumbleTimer.stop();
            rumbleTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
