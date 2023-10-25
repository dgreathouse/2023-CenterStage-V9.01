package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;

public class Claw {
    ServoEx m_right;
    MotorEx m_rotateMotor;
    double m_gripAngle;

    CommandOpMode m_opMode;

    public Claw(CommandOpMode _opMode)
    {
        m_opMode = _opMode;
        initHardware();
    }
    public void initHardware(){
        // TODO: Adjust the servo angles
        m_right = new SimpleServo(m_opMode.hardwareMap,Hw.s_clawSR, 0, 180, AngleUnit.DEGREES);

        m_rotateMotor = new MotorEx(m_opMode.hardwareMap, Hw.s_clawRotate);
       // m_rotateMotor.encoder.setDistancePerPulse(288/360);
        m_rotateMotor.setRunMode(Motor.RunMode.PositionControl);
        m_rotateMotor.encoder.setDirection(Motor.Direction.FORWARD);
        m_rotateMotor.setPositionCoefficient(0.1);
        m_rotateMotor.resetEncoder();

    }
    public void setClawRotateAngle(double _angle, double _speed){
        _angle = MathUtils.clamp(_angle, k.CLAW.RotateDownLimit, k.CLAW.RotateUpLimit);
        m_rotateMotor.setTargetPosition((int)(_angle * k.CLAW.Motor_CountsPDeg));
        m_rotateMotor.set(_speed);

    }
    public int getClawRotateAngle(){
        return m_rotateMotor.getCurrentPosition();
    }
    public void setClawGripAngle(double _angle){
        double angle = _angle / 300.0;
        m_gripAngle = _angle;
        m_right.setPosition(angle);
    }
    public double getClawGripAngle(){
        return m_gripAngle;
    }

}
