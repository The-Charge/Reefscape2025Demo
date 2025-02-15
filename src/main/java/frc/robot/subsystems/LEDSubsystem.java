package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    public static abstract class Pattern {
        /**
         * Use for outputing to leds, prefer to use precalculated values for efficiency reasons
         */
        public abstract void evaluate(AddressableLEDBufferView segment);
    }
    public static abstract class Animation extends Pattern {
        /**
         * Use for updating output values, mostly meant for custom animations
         * MUST ONLY BE CALLED AT MOST ONCE PER UPDATE/FRAME
         */
        public abstract void update();
        /**
         * Use for resetting the animation back to the beginning
         */
        public abstract void reset();
    }
    
    private final AddressableLED leds;
    private final AddressableLEDBuffer buff;
    private final AddressableLEDBufferView segment1, segment2;

    public LEDSubsystem() {
        leds = new AddressableLED(LEDConstants.ledPort);
        buff = new AddressableLEDBuffer(LEDConstants.ledLength);

        segment1 = buff.createView(0, 21);
        segment2 = buff.createView(22, 43);
        
        leds.setLength(LEDConstants.ledLength);
        leds.setColorOrder(LEDConstants.ledColorOrder);

        leds.start();
    }

    @Override
    public void periodic() {
        leds.setData(buff);
    }

    public AddressableLEDBufferView segment1() {
        return segment1;
    }
    public AddressableLEDBufferView segment2() {
        return segment2;
    }
}
