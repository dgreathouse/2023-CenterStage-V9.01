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
        public static final double[] ShoulderAngles = {0,10,15,35,60,85};
        public static final double[] ClawAngles = {0,-10,-10,25,0,-25};



    }
    public static final class CLAW {
        // Claw Grip angles are done in degrees. The maximum degree the servo can move is 300 degrees
        // The minimum is zero. Set the appropriate angle that works for your robot. These values
        // can not be the same for both robots.
        // Options to make the grip work better are.
        // 1. Adjust the numbers so they work good for the top and bottom
        // 2. Add different foam on the pixels to help with the grip types.
        // 3. Arrange the top claw to be at a different angle.
        // 4. ...
        public static final double CloseAngle_22291 = 120;
        public static final double OpenLowerAngle_22291 = 106.5;
        public static final double OpenUpperAngle_22291 = 60.0;
        public static final double OpenAngle_22291 = 60.0;
        public static final double CloseAngle_14623 = 120.0;
        public static final double OpenLowerAngle_14623 = 106.5;
        public static final double OpenUpperAngle_14623 = 60.0;
        public static final double OpenAngle_14623 = 60.0;
        public static final double RotateUpLimit = 25;
        public static final double RotateDownLimit = -25;
        public static final double Motor_CountsPDeg = 0.8;

    }
    public static final class SHOULDER {
        public static final double ThumbRotateUpLimit = 125; //deg
        public static final double ThumbRotateDownLimit = 0;  //deg

        public static final double DownLimit = 0;
        public static final double Motor_CountsPDeg = 2.136;
    }
    public static final class FOREARM {
        public static final double RetractLimit = 0;
        public static final double ExtendLimit = 240; // mm
        public static final double Motor_CountsPmm = 1;
        public static final double ExtentScale_mmPdeg = ExtendLimit/75;
    }
    public static final class LEDS {
        public static final double WhiteServoValue = 0;
        public static final double GreenServoValue = 1;
        public static final double YellowServoValue = 2;
        public static final double PurpleServoValue = 3;
        public static final double OffServoValue = 3;
    }

}
