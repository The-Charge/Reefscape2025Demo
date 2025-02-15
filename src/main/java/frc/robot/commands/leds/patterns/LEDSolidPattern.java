package frc.robot.commands.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem.Pattern;

public class LEDSolidPattern extends Pattern {

    private final LEDPattern solid;

    public LEDSolidPattern(Color col) {
        solid = LEDPattern.solid(col);
    }

    @Override
    public void evaluate(AddressableLEDBufferView segment) {
        solid.applyTo(segment);
    }
}
