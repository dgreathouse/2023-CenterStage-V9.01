package org.firstinspires.ftc.teamcode.Lib;

public class GlobalData {
    public static class DRIVE{
        public static boolean IsVelocityModeEnabled = true;
    }
    public static class MATCH {
        public static TeamPropLocation TeamPropLocation = org.firstinspires.ftc.teamcode.Lib.TeamPropLocation.NONE;
        public static AutoFieldLocation FieldLocation = AutoFieldLocation.BACKDROP;
        public static TeamColor TeamColor = org.firstinspires.ftc.teamcode.Lib.TeamColor.RED;
    }

    public static int TeamNumber = 14623;

    public static boolean ArmAutoEnable = true;


    public static RobotState State = RobotState.AUTONOMOUS;

    public static int tagOfInterest = 7;
    public static double AprilTagBearing = 0;
    public static double AprilTagRange = 0;
    public static double AprilTag_X = 0.01;
    public static double AprilTag_Y = 0.01;
    public static double AprilTag_Z = 0.01;
}
