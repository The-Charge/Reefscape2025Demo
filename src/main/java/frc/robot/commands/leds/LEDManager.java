package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDManager extends Command {
    
    private final LEDSubsystem leds;

    public LEDManager(LEDSubsystem ledSub) {
        leds = ledSub;

        addRequirements(leds);
    }

    @Override
    public void execute() {
        
    }
}
