package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;

public class Forearm {
    private CommandOpMode m_opMode;
    private MotorEx m_motor;

    public Forearm(CommandOpMode _opMode) {
        m_opMode = _opMode;
        initHardware();
    }
    public void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.s_forearm, Motor.GoBILDA.RPM_435);
        m_motor.setRunMode(Motor.RunMode.PositionControl);

    }
    public void setPosition(int _pos){
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setTargetPosition(_pos);
    }
    public int getPosition(){
        return m_motor.getCurrentPosition();
    }
    public void move(double _speed){
        m_motor.setRunMode(Motor.RunMode.RawPower);
        if(getPosition() > k.FOREARM.ExtendLimit || getPosition() <= k.FOREARM.RetractLimit){
            m_motor.set(0);
        }else{
            m_motor.set(_speed);
        }

    }

}
