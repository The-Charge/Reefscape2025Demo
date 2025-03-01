package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import swervelib.parser.PIDFConfig;

public abstract class ClimbConstants {
    
    public static final int leverID = 2;
    public static final double leverMaxVBus = 1;
    public static final double leverMaxCurrent = 80;
    public static final InvertedValue leverInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue leverNeutralMode = NeutralModeValue.Brake;
    
    public static final PIDFConfig leverPIDF = new PIDFConfig(0.1, 0, 0, 0);
    public static final double leverMinPosTicks = 0;
    public static final double leverMaxPosTicks = 40;
    public static final boolean leverUseSoftLimits = false;
    public static final double leverTickToDegConversion = 90 / 28.65;

    public static final double leverRestingDegrees = 3;
    public static final double leverSafeDegrees = 100;
    public static final double leverActiveDegrees = 130;

    public static final int clampID = -1;
    public static final double clampMaxVBus = 0.5;
    public static final double clampMaxCurrent = 40;
    public static final InvertedValue clampInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue clampNeutralMode = NeutralModeValue.Brake;
    
    public static final PIDFConfig clampPIDF = new PIDFConfig(0, 0, 0, 0);
    public static final double clampMinPosTicks = 0;
    public static final double clampMaxPosTicks = Double.MAX_VALUE;
    public static final boolean clampUseSoftLimits = false;
    public static final double clampTickToDegConversion = 1;

    public static final double clampRestingDegrees = 0;
    public static final double clampActiveDegrees = 10;

    public static final double targetThresholdDegrees = 1.5;
    public static final double targetThresholdSeconds = 0.1;
    public static final String leverOverrideDegName = "Climb Lever Target Override (Deg)";
    public static final String clampOverrideDegName = "Climb Clamp Target Override (Deg)";
}
