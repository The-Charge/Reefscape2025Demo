package frc.robot.commands.elev;

import static frc.robot.constants.NoteConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevSubsystem;

public class PlayMusic extends Command {
    
    private final ElevSubsystem elev;

    private boolean isDone;

    public PlayMusic(ElevSubsystem elevSub) {
        elev = elevSub;

        addRequirements(elev);
    }

    @Override
    public void initialize() {
        isDone = false;

        //happy birthday
        new SequentialCommandGroup(
            elev.playTone(G3, quarterNoteSecs * 0.75),
            elev.playTone(G3, quarterNoteSecs / 4),
            elev.playTone(A3, quarterNoteSecs),
            elev.playTone(G3, quarterNoteSecs),
            elev.playTone(C4, quarterNoteSecs),
            elev.playTone(B3, quarterNoteSecs * 2),
            
            elev.playTone(G3, quarterNoteSecs * 0.75),
            elev.playTone(G3, quarterNoteSecs / 4),
            elev.playTone(A3, quarterNoteSecs),
            elev.playTone(G3, quarterNoteSecs),
            elev.playTone(D4, quarterNoteSecs),
            elev.playTone(C4, quarterNoteSecs * 2),

            elev.playTone(G3, quarterNoteSecs * 0.75),
            elev.playTone(G3, quarterNoteSecs / 4),
            elev.playTone(G4, quarterNoteSecs),
            elev.playTone(E4, quarterNoteSecs),
            elev.playTone(C4, quarterNoteSecs),
            elev.playTone(B3, quarterNoteSecs),
            elev.playTone(A3, quarterNoteSecs * 2),

            elev.playTone(F4, quarterNoteSecs * 0.75),
            elev.playTone(F4, quarterNoteSecs / 4),
            elev.playTone(E4, quarterNoteSecs),
            elev.playTone(C4, quarterNoteSecs),
            elev.playTone(D4, quarterNoteSecs),
            elev.playTone(C4, quarterNoteSecs * 3),
            new InstantCommand(() -> {
                isDone = true;
            })
        ).schedule();
    }
    @Override
    public void end(boolean interrupted) {
        elev.playTone(0).schedule();
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
