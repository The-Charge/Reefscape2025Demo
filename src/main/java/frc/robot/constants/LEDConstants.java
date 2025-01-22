package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;

public abstract class LEDConstants {
    
    public static final int ledPort = 0;
    public static final int ledLength = 44;
    public static final boolean swapRedGreen = true; //use if red and green channels need to be swapped

    public static final LEDPattern colorTest = LEDPattern.steps(Map.ofEntries(
        Map.entry(0d, LEDSubsystem.modColor(new Color("#ff0000"))),
        Map.entry(1 / 12.0, LEDSubsystem.modColor(new Color("#ff8800"))),
        Map.entry(2 / 12.0, LEDSubsystem.modColor(new Color("#ffff00"))),
        Map.entry(3 / 12.0, LEDSubsystem.modColor(new Color("#88ff00"))),
        Map.entry(4 / 12.0, LEDSubsystem.modColor(new Color("#00ff00"))),
        Map.entry(5 / 12.0, LEDSubsystem.modColor(new Color("#00ff88"))),
        Map.entry(6 / 12.0, LEDSubsystem.modColor(new Color("#00ffff"))),
        Map.entry(7 / 12.0, LEDSubsystem.modColor(new Color("#0088ff"))),
        Map.entry(8 / 12.0, LEDSubsystem.modColor(new Color("#0000ff"))),
        Map.entry(9 / 12.0, LEDSubsystem.modColor(new Color("#8800ff"))),
        Map.entry(10 / 12.0, LEDSubsystem.modColor(new Color("#ff00ff"))),
        Map.entry(11 / 12.0, LEDSubsystem.modColor(new Color("#ff0088")))
    ));
    public static final Color chargeGreen = new Color("#008800");
    public static final Color chargeGold = new Color("#ffaa00");
}
