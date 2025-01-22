package frc.robot.constants;

public abstract class VisionConstants {
    public static class LLReefConstants
  {
    public static final String LL_NAME = "limelight-reef";
    public static final double ARRAY_NUM = 0;
    //pipelines
    public static final double VIEWPORT_PIPELINE = 0;
    public static final double APRILTAG_PIPELINE = 1;

    //limelight pose in robotspace
  }

  public static class LLFunnelConstants
  {
    public static final String LL_NAME = "limelight-funnel";
    public static final double ARRAY_NUM = 1;
    //pipelines
    public static final double VIEWPORT_PIPELINE = 0;
    public static final double APRILTAG_PIPELINE = 1;

    //
  }
  public static class ApriltagConstants
  {

    //Tag ids for each tag
    public static final double LOW_RED_CORALSTATION_ID = 1;
    public static final double HIGH_RED_CORALSTATION_ID = 2;
    public static final double BLUE_PROCESSOR_ID = 3;
    public static final double RIGHT_BLUE_BARGE_ID = 4;
    public static final double RIGHT_RED_BARGE_ID = 5;
    public static final double BOTTOMRIGHT_RED_REEF_ID = 6;
    public static final double RIGHT_RED_REEF_ID = 7;
    public static final double TOPRIGHT_RED_REEF_ID = 8;
    public static final double TOPLEFT_RED_REEF_ID = 9;
    public static final double LEFT_RED_REEF_ID = 10;
    public static final double BOTTOMLEFT_RED_REEF_ID = 11;
    public static final double BOTTOM_BLUE_CORALSTATION_ID = 12;
    public static final double TOP_BLUE_CORALSTATION_ID = 13;
    public static final double LEFT_BLUE_BARGE_ID = 14;
    public static final double LEFT_RED_BARGE_ID = 15;
    public static final double RED_PROCESSOR_TAG_ID = 16;
    public static final double BOTTOMLEFT_BLUE_REEF_ID = 17;
    public static final double LEFT_BLUE_REEF_ID = 18;
    public static final double TOPLEFT_BLUE_REEF_ID = 19;
    public static final double TOPRIGHT_BLUE_REEF_ID = 20;
    public static final double RIGHT_BLUE_REEF_ID = 21;
    public static final double BOTTOMRIGHT_BLUE_REEF_ID = 22;
  }
}
