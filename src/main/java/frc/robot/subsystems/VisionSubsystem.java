package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ApriltagConstants;
import frc.robot.Constants.LLReefConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase{
    public String limelightname = LLReefConstants.REEF_LL_NAME;   //first camera initialiation
    public double tx;               //X-offset
    public double ty;               //Y-offset
    public double tv;               //Target Identification
    public double ta;               //Area of tag
    public double tid;              //Tag id
    public double tl;               //latency contribution
    public double cl;               //Capture pipeline latency
    public double getpipe;          //get current pipeline
    public double limelightlatency; //tl + cl
    public double distance;         //distance to target
    public Pose2d robotpose;        //Robot in Fieldspace (blue side)
    public double prevtag;

    public VisionSubsystem(){
    
    }

    @Override
    public void periodic(){
      // This method will be called once per scheduler run
      updateLimelightTracking();
    }
  
    //updates limelight tracked values and puts on SmartDashboard
    public void updateLimelightTracking(){ 
        //Read general target values
        tv = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("ta").getDouble(0);
        tid = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("tid").getDouble(0);
        tl = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("tl").getDouble(0);
        cl = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("cl").getDouble(0);
        getpipe = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("getpipe").getDouble(0);

        
    }

    public void setPipeline(double index){
      NetworkTableInstance.getDefault().getTable(limelightname).getEntry("pipeline").setNumber(index);
    }

    public void setRobotOrientation(double robotYaw){
      LimelightHelpers.SetRobotOrientation(limelightname, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    
}