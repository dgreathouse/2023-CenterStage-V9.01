package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;

public class DroneSubsystem extends SubsystemBase {
    MotorEx m_motor;
    private CommandOpMode m_opMode;

    public DroneSubsystem(CommandOpMode _opMode){
        m_opMode = _opMode;
        initHardware();
    }
    private void initHardware() {
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.s_Drone, Motor.GoBILDA.RPM_1620);
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void spin(double _speed){
        m_motor.set(_speed);
    }
    @Override
    public void periodic() {

    }
}
