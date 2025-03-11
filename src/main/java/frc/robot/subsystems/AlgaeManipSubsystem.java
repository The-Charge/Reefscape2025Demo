package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeManipConstants;
import frc.robot.constants.TelemetryConstants;
import swervelib.parser.PIDFConfig;

public class AlgaeManipSubsystem extends SubsystemBase {
    
    private final SparkMax pivot, roller;
    
    private double targetTicks;
    private int targetCounter;
    private boolean pivotAtTarget;

    public AlgaeManipSubsystem() {
        pivot = new SparkMax(AlgaeManipConstants.pivotID, MotorType.kBrushless);
        roller = new SparkMax(AlgaeManipConstants.rollerID, MotorType.kBrushless);

        configureMotor(pivot, AlgaeManipConstants.pivotInverted, AlgaeManipConstants.pivotIdle, AlgaeManipConstants.pivotCurrentLimit);
        configureMotor(roller, AlgaeManipConstants.rollerInverted, AlgaeManipConstants.rollerIdle, AlgaeManipConstants.rollerCurrentLimit);

        setPID(pivot, AlgaeManipConstants.pivotPID);

        pivot.set(0);
        roller.set(0);
        targetTicks = 0;
        targetCounter = 0;
        pivotAtTarget = true;
    }

    @Override
    public void periodic() {
        targetCheck();

        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putNumber("AlgaeManip Pivot Pos (Deg)", getPivotDegrees());
            SmartDashboard.putNumber("AlgaeManip Pivot Target (Deg)", targetTicks * AlgaeManipConstants.pivotTicksToDegConversion);
            SmartDashboard.putBoolean("AlgaeManip pivotAtTarget", pivotAtTarget());
            SmartDashboard.putNumber("AlgaeManip Roller VBus", roller.get());

            SmartDashboard.putNumber("AlgaeManip Pivot Pos (Ticks)", getPivotTicks());
            SmartDashboard.putNumber("AlgaeManip Pivot Target (Deg)", targetTicks);

            SmartDashboard.putNumber("AlgaeManip Pivot VBus", pivot.get());
            SmartDashboard.putNumber("AlgaeManip Pivot Current", pivot.getOutputCurrent());
            SmartDashboard.putNumber("AlgaeManip Roller Current", roller.getOutputCurrent());
            if(getCurrentCommand() == null)
                SmartDashboard.putString("AlgaeManip RunningCommand", "None");
            else
                SmartDashboard.putString("AlgaeManip RunningCommand", getCurrentCommand().getName());
        }
    }

    public void setPivotDegrees(double deg) {
        setTicks(pivot, deg / AlgaeManipConstants.pivotTicksToDegConversion, AlgaeManipConstants.pivotMinTicks, AlgaeManipConstants.pivotMaxTicks);
    }
    public void rollerVBus(double speed) {
        roller.set(speed);
    }
    public void rollerStop() {
        rollerVBus(0);
    }

    public double getPivotDegrees() {
        return getPivotTicks() * AlgaeManipConstants.pivotTicksToDegConversion;
    }
    public boolean pivotAtTarget() {
        return pivotAtTarget;
    }

    private void configureMotor(SparkMax m, boolean inverted, IdleMode idle, int currentLimit) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idle);
        config.smartCurrentLimit(currentLimit);
        config.inverted(inverted);

        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    private void setPID(SparkMax m, PIDFConfig pidf) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pidf(pidf.p, pidf.i, pidf.d, pidf.f);
        config.closedLoop.outputRange(-1, 1);

        m.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters); //don't reset other parameters
    }
    private void setTicks(SparkMax m, double ticks, double min, double max) {
        targetTicks = MathUtil.clamp(ticks, min, max);

        m.getClosedLoopController().setReference(ticks, ControlType.kPosition);
        resetTargetCounter();
    }
    private double getPivotTicks() {
        return pivot.getEncoder().getPosition();
    }
    private void resetTargetCounter() {
        targetCounter = 0;
        pivotAtTarget = false;
    }
    private void targetCheck() {
        double err = Math.abs(targetTicks - getPivotTicks());
        if(err <= AlgaeManipConstants.targetThresholdDeg / AlgaeManipConstants.pivotTicksToDegConversion)
            targetCounter++;
        else
            resetTargetCounter();
        
        if(targetCounter >= AlgaeManipConstants.targetThresholdSeconds * 50)
            pivotAtTarget = true;
        else
            pivotAtTarget = false;
    }
}
