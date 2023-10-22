package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class k {
    public static final class DRIVE {
        public static final double CPR = Motor.GoBILDA.RPM_435.getCPR();
        public static final double WheelDiameter_in = 3.78;
        public static final double WheelCircumference_in = Math.PI * WheelDiameter_in;
        public static final double InchPerCount = WheelCircumference_in / CPR;
        public static final double[] DriveDistance =       { 00.00,01.00,02.00,03.00,04.00,05.00,12.00,24.00,36.00,48.00,100.00 };
        public static final double[] DriveTime_Speed60 = { 00.00,01.00,02.00,03.00,04.00,05.00,12.00,24.00,36.00,48.00,100.00 };

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


    }
    public static final class CLAW {
        public static final double CloseAngle = 0.0;
        public static final double OpenLowerAngle = 1.0;
        public static final double OpenUpperAngle = 2.0;
        public static final double RotateUpLimit = 0;
        public static final double RotateDownLimit = 100;

    }
    public static final class SHOULDER {
        public static final double RotateUpLimit = 90; //deg
        public static final double RotateDownLimit = -36;  //deg
    }
    public static final class FOREARM {
        public static final double RetractLimit = 0;
        public static final double ExtendLimit = 240; // mm
    }
    public static final class LEDS {
        public static final double WhiteServoValue = 0;
        public static final double GreenServoValue = 1;
        public static final double YellowServoValue = 2;
        public static final double PurpleServoValue = 3;
        public static final double OffServoValue = 3;
    }

}
