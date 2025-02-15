package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.HeadConstants;

public class HeadSubsystem extends SubsystemBase {

    private DigitalInput headSensor1;
    private DigitalInput headSensor2;
    private SparkMax headLeft;
    private SparkMax headRight;
    private boolean hasCoral = false;
    private TimeOfFlight ToFSensor;
    
    public HeadSubsystem() {
        headSensor1 = new DigitalInput(HeadConstants.sensorId1);
        headSensor2 = new DigitalInput(HeadConstants.sensorId2);

        headLeft = new SparkMax(HeadConstants.leftId, MotorType.kBrushless);
        headRight = new SparkMax(HeadConstants.rightId, MotorType.kBrushless);

        ToFSensor = new TimeOfFlight(1);
        configureMotor(headLeft);
        configureMotor(headRight);

        headLeft.set(0);
        headRight.set(0);

        createSensorTriggers();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Head VBus L", headLeft.get());
        SmartDashboard.putNumber("Head VBus R", headRight.get());
        SmartDashboard.putBoolean("Head Sensor 1", getHeadSensor1());
        SmartDashboard.putBoolean("Head Sensor 2", getHeadSensor2());
        SmartDashboard.putNumber("TOF thing", ToFSensor.getRange());
    }

    public void flywheelVBus(double percent) {
        headLeft.set(percent);
        headRight.set(percent);
    }
    public void stop() {
        headLeft.set(0);
        headRight.set(0);
    }

    public boolean getHeadSensor1() {
        return headSensor1.get(); 
    }
    public boolean getHeadSensor2() {
        return headSensor2.get();
    }
    public boolean getHasCoral() {
        return hasCoral;
    }

    private void configureMotor(SparkMax m) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(HeadConstants.currentLimit);

        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    private void createSensorTriggers() {
        new Trigger(() -> getHeadSensor1()).onTrue(new InstantCommand(() -> {
            hasCoral = true;
        }));
        new Trigger(() -> getHeadSensor2()).onTrue(new InstantCommand(() -> {
            hasCoral = false;
        }));
    }
}
