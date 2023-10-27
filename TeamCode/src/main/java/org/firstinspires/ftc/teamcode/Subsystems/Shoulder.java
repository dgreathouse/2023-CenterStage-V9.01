package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;

public class Shoulder {

    private  MotorEx m_motor;
    private CommandOpMode m_opMode;
    private PIDController rotPID;
    public Shoulder(CommandOpMode _opMode) {
        m_opMode = _opMode;
        initHardware();
    }

    public void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.s_SH, MotorEx.GoBILDA.RPM_435);
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.setInverted(false);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rotPID = new PIDController(0.06,0.01,0);
        rotPID.setTolerance(1);
        rotPID.setIntegrationBounds(-0.5,0.5);
        rotPID.reset();
    }
    public void setAngle(double _ang){
        double rot = rotPID.calculate(m_motor.getCurrentPosition() / k.SHOULDER.Motor_CountsPDeg, _ang);
        m_motor.set(rot);
    }
    public double getAngle(){
       return  m_motor.getCurrentPosition() / k.SHOULDER.Motor_CountsPDeg;
    }

}
