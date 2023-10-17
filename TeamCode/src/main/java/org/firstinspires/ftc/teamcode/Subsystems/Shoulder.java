package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;

public class Shoulder {

    private  MotorEx m_motor;
    private CommandOpMode m_opMode;

    public Shoulder(CommandOpMode _opMode) {
        m_opMode = _opMode;
        initHardware();
    }

    public void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.s_SH, MotorEx.GoBILDA.RPM_435);
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setInverted(false);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.setPositionCoefficient(0.10);
    }
    public void sePosition(int _pos){
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setTargetPosition(_pos);
    }
    public int getPosition(){
       return  m_motor.getCurrentPosition();
    }
    public void move(double _speed){
        m_motor.setRunMode(Motor.RunMode.RawPower);
        // TODO set constants to k.java values
        if(getPosition() > k.SHOULDER.RotateUpLimit || getPosition() <= k.SHOULDER.RotateDownLimit) {
            m_motor.set(0);
        }else {
            m_motor.set(_speed);
        }


    }
}
