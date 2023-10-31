package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class k {
    public static final class SYSTEM{
        public static final boolean isLoopRateLimited = false;
    }
    public static final class DRIVE {
        public static final double CPR = Motor.GoBILDA.RPM_435.getCPR();
        public static final double WheelDiameter_in = 3.78;
        public static final double WheelCircumference_in = Math.PI * WheelDiameter_in;
        public static final double InchPerCount = WheelCircumference_in / CPR;
        public static final double[] DriveDistance =       { 00.00,01.00,02.00,03.00,04.00,05.00,12.00,24.00,36.00,48.00,100.00 };
        public static final double[] DriveTime_Speed60 = { 00.00,01.00,02.00,03.00,04.00,05.00,12.00,24.00,36.00,48.00,100.00 };
        public static final double Rot_P = 0.0075;
        public static final double Rot_I = 0.05;

    }
    public static final class CAMERA {
        public static final double FEET_PER_METER = 3.28084;
        public static final double fx = 1420.7594;
        public static final double fy = 1420.9965;
        public static final double cx = 630.8343;
        public static final double cy = 381.3086;
        public static final double tagsize = 0.166;
        public static  int numFramesWithoutDetection = 0;

        public static final float DECIMATION_HIGH = 3;
        public static final float DECIMATION_LOW = 2;
        public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        public static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
        public static final int ID_TAG_OF_INTEREST = 1; // Tag ID 1 from the 36h11 family
    }
    public static final class ARM {
        public static final double TeamPropMinimumDistance_mm = 20;
        public static TeamPropLocation TeamPropLoc = TeamPropLocation.CENTER;
        public static final double[] ShoulderAngles = {0,10,15,35,60,85};
        public static final double[] ClawAngles = {0,-10,-10,25,0,-25};



    }
    public static final class CLAW {

        public static final double RotateUpLimit = 25;
        public static final double RotateDownLimit = -25;
        public static final double Motor_CountsPDeg = 288.0/360.0;

        public static final double CloseAngle_22291 = 115;
        public static final double OpenLowerAngle_22291 = 101.5;
        public static final double OpenUpperAngle_22291 = 60.0;
        public static final double OpenAngle_22291 = 60.0;

        public static final double CloseAngle_14623 = 115.0;
        public static final double OpenLowerAngle_14623 = 101.5;
        public static final double OpenUpperAngle_14623 = 60.0;
        public static final double OpenAngle_14623 = 60.0;

    }
    public static final class SHOULDER {

        public static final double AngleVertical_22291 = 125.0;
        public static final double AngleBackdropUpLimit_22291 = 85;
        public static final double AngleStraight_22291 = 35.0;
        public static final double AngleStack_5_22291 = 10.0;
        public static final double AngleStack_3_22291 = 10.0;
        public static final double AngleFloor_22291 = 0.0;

        public static final double AngleVertical_14623 = 125.0;
        public static final double AngleBackdropUpLimit_14623 = 85;
        public static final double AngleStraight_14623 = 35.0;
        public static final double AngleStack_5_14623 = 10.0;
        public static final double AngleStack_3_14623 = 10.0;
        public static final double AngleFloor_14623 = 0.0;

        public static final double ThumbRotateUpLimit = 125; //deg
        public static final double ThumbRotateDownLimit = 0;  //deg
        // 16 tooth to 48 tooth or 3:1/0.333 435 RPM motor with a
        // 384.5 counts/rev
        // at shoulder that is 384.5 * 3 = 1153.5 counts per Rev of shoulder
        // That means there is 1153.5/360 = 3.204 counts/deg
        public static final double Motor_CountsPDeg = 3.204;
    }
    public static final class FOREARM {
        public static final double RetractLimit = 5;
        public static final double ExtendLimit = 230; // mm
        public static final double Motor_CountsPmm = 6;
       // public static final double ExtentScale_mmPdeg = ExtendLimit/(85 - 35);
    }


}
