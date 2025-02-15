package frc.robot.commands.leds.patterns;

import java.util.Map;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

public class LEDStepPattern extends Pattern {

    private final LEDPattern chase;

    public LEDStepPattern(Map<? extends Number, Color> cols, LinearVelocity scrollSpeed) {
        chase = LEDPattern.steps(cols)
            .scrollAtAbsoluteSpeed(scrollSpeed, LEDConstants.ledSpacing);
    }

    @Override
    public void evaluate(AddressableLEDBufferView segment) {
        chase.applyTo(segment);
    }
}
