package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;

public class Claw {
    //ServoEx m_right;
    MotorEx m_rotateMotor;

    PIDController m_pid;
    CommandOpMode m_opMode;
    double clawAngle = 0;
    public Claw(CommandOpMode _opMode)
    {
        m_opMode = _opMode;
        initHardware();
    }
    private void initHardware(){
        //m_right = new SimpleServo(m_opMode.hardwareMap,Hw.ClawServoRotate, 0, 300, AngleUnit.DEGREES);

        m_rotateMotor = new MotorEx(m_opMode.hardwareMap, Hw.ClawRotateMotor);
        m_rotateMotor.setRunMode(Motor.RunMode.RawPower);
        m_rotateMotor.encoder.setDirection(Motor.Direction.FORWARD);
        m_rotateMotor.resetEncoder();

        m_pid = new PIDController(0.01,0.01,0.0);
        m_pid.setTolerance(1.6);
    }
    public void setClawRotateAngle(double _angle){

        //_angle = MathUtils.clamp(_angle, k.CLAW.RotateDownLimit, k.CLAW.RotateUpLimit);
        clawAngle = _angle;
        double rot = m_pid.calculate(getClawRotateAngle(),_angle);

        m_rotateMotor.set(rot);

    }
    public double getClawRotateAngle(){
        return m_rotateMotor.getCurrentPosition() / k.CLAW.Motor_CountsPDeg;
    }
    public double getClawRequestedAngle(){
        return clawAngle;
    }



}
