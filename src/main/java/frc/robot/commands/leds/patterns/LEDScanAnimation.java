package frc.robot.commands.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem.Animation;

public class LEDScanAnimation extends Animation {

    private final Color col;
    private final double speed;
    private long time;

    public LEDScanAnimation(Color color, double speedMult) {
        col = color;
        speed = speedMult;

        time = 0;
    }

    @Override
    public void update() {
        time++;
    }
    @Override
    public void reset() {
        time = 0;
    }
    @Override
    public void evaluate(AddressableLEDBufferView segment) {
        final double pulseWidth = 10; //width of visible pulse

        final double offsetRange = segment.getLength() - pulseWidth;
        double offset = (offsetRange * 0.5) * (Math.sin(((Math.PI * time * speed) / offsetRange) - (0.5 * Math.PI)) + 1);

        for(int i = 0; i < segment.getLength(); i++) {
            double strength = 0;
            if(offset < i && i < offset + pulseWidth) {
                strength = Math.sin((Math.PI / pulseWidth) * (i - offset));
            }

            Color pixel = new Color(
                col.red * strength,
                col.green * strength,
                col.blue * strength
            );

            segment.setLED(i, pixel);
        }
    }
}
