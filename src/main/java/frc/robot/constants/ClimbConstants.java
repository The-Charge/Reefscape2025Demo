package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import swervelib.parser.PIDFConfig;

public abstract class ClimbConstants {
    
    public static final int motorID = 2;
    public static final double maxVBus = 0.6;
    public static final double maxCurrent = 80;
    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    
    public static final PIDFConfig pidf = new PIDFConfig(0.1, 0, 0, 0);
    public static final double minPosTicks = 0;
    public static final double maxPosTicks = 40;
    public static final double tickToDegConversion = 90 / 28.65;
    public static final double targetThresholdDegrees = 1.5;
    public static final double targetThresholdSeconds = 0.1;

    public static final double restingDegrees = 3;
    public static final double safeDegrees = 100;
    public static final double activeDegrees = 130;

    public static final String overrideDegName = "Climb Target Override (Deg)";
    public static final String overrideTicksName = "Climb Target Override (Ticks)";
}
