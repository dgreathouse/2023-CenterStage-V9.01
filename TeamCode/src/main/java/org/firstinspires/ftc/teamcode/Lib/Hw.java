package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class Hw {
    // Control Hub
    public static String s_fl = "l"; // Drive Left Motor
    public static String s_fr = "r"; // Drive Right Motor
    public static String s_b = "b";  // Drive Back Motor
    public static String s_SH = "s"; // Shoulder Motor
    public static String s_clawSR = "csr";
    public static String s_distance = "dis";
    public static String s_distanceServo = "disS";
    public static String s_ledTop = "lt";
    public static String s_ledBot = "lb";
    // Expansion hub
    public static String s_clawRotate = "crm";
    public static String s_Drone = "d";
    public static String s_forearm = "f";

    public static GamepadEx s_gpOperator;
    public static GamepadEx s_gpDriver;
    public static IMU s_imu;
    CommandOpMode m_opMode;
    public Hw(CommandOpMode _opMode) {
        m_opMode = _opMode;
    }
    public void init(){

        s_gpDriver = new GamepadEx(m_opMode.gamepad1);
        s_gpOperator = new GamepadEx(m_opMode.gamepad2);
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        s_imu = m_opMode.hardwareMap.get(BHI260IMU.class,"imu");

      //  imu.init();


        m_opMode.telemetry.addData(">", "Hardware Initialized");
    }
}
