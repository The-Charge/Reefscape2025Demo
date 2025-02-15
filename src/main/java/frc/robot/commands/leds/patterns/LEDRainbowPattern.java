package frc.robot.commands.leds.patterns;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

public class LEDRainbowPattern extends Pattern {

    private final LEDPattern rainbow;

    public LEDRainbowPattern(LinearVelocity scrollSpeed) {
        rainbow = LEDPattern.rainbow(255, 255)
            .scrollAtAbsoluteSpeed(scrollSpeed, LEDConstants.ledSpacing);
    }

    @Override
    public void evaluate(AddressableLEDBufferView segment) {
        rainbow.applyTo(segment);
    }
}
