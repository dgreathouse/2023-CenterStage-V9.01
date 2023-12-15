package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import java.util.ArrayList;

public class k {
    public static final class FIELD{
        public static final double aprilTagWalltoStackDistanceX_mm = 140;
        public static final double PixelLength_mm = 76.2;
    }
    public static final class SYSTEM{
        public static final boolean isLoopRateLimited = false;
        public static final double CameraToStack5_z = 0;
        public static final double CameraToStack3_z = 0;
        public static final double CameraToCenter_x = 0;

    }
    public static final class DRIVE {
        public static final double CPR = Motor.GoBILDA.RPM_435.getCPR();
        public static final double WheelDiameter_in = 3.78;
        public static final double WheelCircumference_in = Math.PI * WheelDiameter_in;
        public static final double InchPerCount = WheelCircumference_in / CPR;
        public static final double[] DriveDistance =       { 00.00,01.00,02.00,03.00,04.00,05.00,12.00,24.00,36.00,48.00,100.00 };
        public static final double[] DriveTime_Speed60 = { 00.00,01.00,02.00,03.00,04.00,05.00,12.00,24.00,36.00,48.00,100.00 };
        public static final double Rot_P = 0.005;
        public static final double Rot_I = 0.05;
        public static final double Drive_P = 1.0;
        public static final double Drive_I = 0;
        public static double DriveSpeedScale = 1.0;
        public static boolean AutoDriveRampEnabled = true;
        public static double ACHIEVABLE_MAX_TICKS_PER_SECOND = 2781.1;

    }
    public static final class CAMERA {
        public static final double FEET_PER_METER = 3.28084;
//        public static final double fx = 1420.7594;
//        public static final double fy = 1420.9965;
//        public static final double cx = 630.8343;
//        public static final double cy = 381.3086;
        public static final double fx = 1121.6864;
        public static final double fy = 1133.7887;
        public static final double cx = 635.2146;
        public static final double cy = 465.3686;
        //public static final double tagsize = 0.0508;  // 2 inch
        public static final double tagsize = 0.127;  // 5 inch
        //public static final double tagsize = 0.1524;  // 6 inch
        public static  int numFramesWithoutDetection = 0;

        public static final float DECIMATION_HIGH = 3;
        public static final float DECIMATION_LOW = 2;
        public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        public static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    }
    public static final class ARM {
        public static final double TeamPropMinimumDistance_mm = 700;
        public static TeamPropLocation TeamPropLoc = TeamPropLocation.CENTER;
        public static final double[] ShoulderAngles = {0,8,10,35,60,85};
        public static final double[] ClawAngles = {0,-4,10,35,0,-6};

    }
    public static final class CLAW {

        public static final double RotateUpLimit = 25;
        public static final double RotateDownLimit = -25;
        public static final double Motor_CountsPDeg = 288.0/360.0;

        public static final double CloseAngle_22291 = 281;
        public static final double OpenLowerAngle_22291 = 265;
        public static final double OpenUpperAngle_22291 = 240;
        public static final double OpenAngle_22291 = 220;

        public static final double CloseAngle_14623 = 130.50;  // Left Trigger
        public static final double OpenLowerAngle_14623 = 121.0; // Right Trigger
        public static final double OpenUpperAngle_14623 = 110.0; // Right Bumper
        public static final double OpenAngle_14623 = 60.0; // Left Bumper


    }
    public static final class SHOULDER {

        public static final double AngleVertical_22291 = 125.0;
        public static final double AngleBackdropUpLimit_22291 = 85;
        public static final double AngleStraight_22291 = 35.0;
        public static final double AngleBackdrop_22291 = 60.0;
        public static final double AngleStack_5_22291 = 10.0;
        public static final double AngleStack_3_22291 = 3.0;
        public static final double AngleFloor_22291 = 0.0;

        public static final double AngleVertical_14623 = 125.0;
        public static final double AngleBackdropUpLimit_14623 = 85;
        public static final double AngleStraight_14623 = 35.0;
        public static final double AngleBackdrop_14623 = 60.0;
        public static final double AngleStack_5_14623 = 11.0;
        public static final double AngleStack_3_14623 = 8.0;
        public static final double AngleFloor_14623 = 0.0;

        public static final double ThumbRotateUpLimit = 125; //deg
        public static final double ThumbRotateDownLimit = 0;  //deg
        // 16 tooth to 48 tooth or 3:1/0.333 435 RPM motor with a
        // 384.5 counts/rev
        // at shoulder that is 384.5 * 3 = 1153.5 counts per Rev of shoulder
        // That means there is 1153.5/360 = 3.204 counts/deg
        public static final double Motor_CountsPDeg = 3.204;
        public static double RotationPID_Min = -0.16;
        public static double RotationPID_Max = 2;
    }
    public static final class FOREARM {
        public static final double RetractLimit = 0;
        public static final double ExtendLimit = 210; // mm
        // 8mm lead screw means 8mm per rotation.
        // Gears are 36:60, Counts/Rev of motor = 103.8 for 1620 RPM Motor
        public static final double Motor_CountsPmm = 21.625;
       // public static final double ExtentScale_mmPdeg = ExtendLimit/(85 - 35);
    }


}
