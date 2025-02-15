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
        final double width = 10; //fixed width

        double offset = ((Math.sin((Math.PI * time * speed) / (segment.getLength() - width) - 0.5 * Math.PI) + 1) * 0.5) * (segment.getLength() - width);
        for(int i = 0; i < segment.getLength(); i++) {
            double strength = 0;
            if(offset < i && i < width + offset) {
                strength = Math.sin(Math.PI * (i - offset) / width);
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
