package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HeadConstants;
import frc.robot.constants.TelemetryConstants;

public class HeadSubsystem extends SubsystemBase {

    private TimeOfFlight branchSensor;
    private TimeOfFlight coralSensor;
    private TimeOfFlight backSensor;
    private TimeOfFlight frontSensor;
    private SparkMax headLeft;
    private SparkMax headRight;
    private boolean hasCoral = false;
    
    public HeadSubsystem() {
        branchSensor = new TimeOfFlight(HeadConstants.branchSensorId);
        coralSensor = new TimeOfFlight(HeadConstants.coralSensorId);
        backSensor = new TimeOfFlight(HeadConstants.backSensorId);
        frontSensor = new TimeOfFlight(HeadConstants.frontSensorId);

        branchSensor.setRangingMode(RangingMode.Long, HeadConstants.longSensorSampleTime);
        coralSensor.setRangingMode(RangingMode.Short, HeadConstants.shortSensorSampleTime);
        backSensor.setRangingMode(RangingMode.Long, HeadConstants.longSensorSampleTime);
        frontSensor.setRangingMode(RangingMode.Long, HeadConstants.longSensorSampleTime);

        headLeft = new SparkMax(HeadConstants.leftId, MotorType.kBrushless);
        headRight = new SparkMax(HeadConstants.rightId, MotorType.kBrushless);

        configureMotor(headLeft, HeadConstants.leftReversed);
        configureMotor(headRight, HeadConstants.rightReversed);

        headLeft.set(0);
        headRight.set(0);
    }

    @Override
    public void periodic() {
        hasCoral = getCoralSensor();

        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putNumber("Head VBus L", headLeft.get());
            SmartDashboard.putNumber("Head VBus R", headRight.get());
            SmartDashboard.putBoolean("Head HasCoral", getHasCoral());
            
            SmartDashboard.putBoolean("Head Branch Sensor (Bool)", getBranchSensor());
            SmartDashboard.putBoolean("Head Coral Sensor (Bool)", getCoralSensor());

            SmartDashboard.putNumber("Head Branch Sensor (mm)", branchSensor.getRange());
            SmartDashboard.putNumber("Head Coral Sensor (mm)", coralSensor.getRange());
            SmartDashboard.putNumber("Head Back Sensor (mm)", backSensor.getRange());
            SmartDashboard.putNumber("Head Front Sensor (mm)", frontSensor.getRange());
    
            SmartDashboard.putNumber("Head Current L", headLeft.getOutputCurrent());
            SmartDashboard.putNumber("Head Current R", headRight.getOutputCurrent());
            if(getCurrentCommand() == null)
                SmartDashboard.putString("Head RunningCommand", "None");
            else
                SmartDashboard.putString("Head RunningCommand", getCurrentCommand().getName());
        }
    }

    public void flywheelLeft(double percent) {
        headLeft.set(percent);
    }
    public void flywheelRight(double percent) {
        headRight.set(percent);
    }
    public void stop() {
        headLeft.set(0);
        headRight.set(0);
    }

    public boolean getBranchSensor() {
        return branchSensor.getRange() <= HeadConstants.branchActivationDist;
    }
    public boolean getCoralSensor() {
        return coralSensor.getRange() <= HeadConstants.coralActivationDist;
    }
    public double getBackDistance() {
        return backSensor.getRange();
    }
    public double getFrontDistance() {
        return frontSensor.getRange();
    }
    public boolean getHasCoral() {
        return hasCoral;
    }

    public void recheckHasCoral() {
        hasCoral = getCoralSensor();
    }

    private void configureMotor(SparkMax m, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(HeadConstants.idleMode);
        config.smartCurrentLimit(HeadConstants.currentLimit);
        config.inverted(inverted);

        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
