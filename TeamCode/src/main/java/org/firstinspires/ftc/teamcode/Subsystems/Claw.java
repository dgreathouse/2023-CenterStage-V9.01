package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.Hw;

public class Claw {
    ServoEx m_right;
    MotorEx m_rotateMotor;


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
    public void setClawRotateAngle(int _angle, double _speed){
        m_rotateMotor.setTargetPosition(_angle);
        m_rotateMotor.set(_speed);

    }
    public int getClawRotateAngle(){
        return m_rotateMotor.getCurrentPosition();
    }
    public void setClawGripAngle(double _angle){
        m_right.setPosition(_angle);
    }

}
