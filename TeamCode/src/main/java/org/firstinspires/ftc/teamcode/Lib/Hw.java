package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class Hw {
    // Control Hub
    public static String DriveFrontLeft = "l"; // Drive Left Motor
    public static String DriveFrontRight = "r"; // Drive Right Motor
    public static String DriveBack = "b";  // Drive Back Motor
    public static String ForearmMotor = "f";
    public static String ClawServoRotate = "csr";
    public static String DistanceSensor = "dis";

    // Expansion hub
    public static String ClawRotateMotor = "crm";
    public static String DroneMotor = "d";
    public static String ShoulderMotor = "s"; // Shoulder Motor

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

        s_imu = m_opMode.hardwareMap.get(BHI260IMU.class,"imu");
    }
    public void initDriveHardware(CommandOpMode _opMode, MotorEx _l, MotorEx _r, MotorEx _b) {
        _l = new MotorEx(m_opMode.hardwareMap, Hw.DriveFrontLeft, Motor.GoBILDA.RPM_435);
        _l.setInverted(true);
        _l.setRunMode(Motor.RunMode.VelocityControl);
        _l.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        _l.setDistancePerPulse(k.DRIVE.InchPerCount);
        _l.encoder.setDirection(Motor.Direction.FORWARD);
        _l.setVeloCoefficients(1.0,0.01,0);

        _r = new MotorEx(m_opMode.hardwareMap, Hw.DriveFrontRight, Motor.GoBILDA.RPM_435);
        _r.setInverted(true);
        _r.setRunMode(Motor.RunMode.VelocityControl);
        _r.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        _r.setDistancePerPulse(k.DRIVE.InchPerCount);
        _r.encoder.setDirection(Motor.Direction.REVERSE);
        _r.setVeloCoefficients(1.0,0.01,0);

        _b = new MotorEx(m_opMode.hardwareMap, Hw.DriveBack, Motor.GoBILDA.RPM_435);
        _b.setInverted(true);
        _b.setRunMode(Motor.RunMode.VelocityControl);
        _b.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        _b.setDistancePerPulse(k.DRIVE.InchPerCount);
        _b.encoder.setDirection(Motor.Direction.REVERSE);
        _b.setVeloCoefficients(1.0,0.01,0);
    }
}
