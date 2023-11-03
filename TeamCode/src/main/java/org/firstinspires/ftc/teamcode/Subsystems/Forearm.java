package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;

public class Forearm {
    private CommandOpMode m_opMode;
    private MotorEx m_motor;
    private PIDController m_pid;
    private double m_speed = 0;
    private double m_actualSpeed = 0;
    public Forearm(CommandOpMode _opMode) {
        m_opMode = _opMode;
        initHardware();
    }
    private void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.ForearmMotor, Motor.GoBILDA.RPM_1620);
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.resetEncoder();
        m_pid = new PIDController(0.05, 0.15, 0);
        m_pid.setTolerance(0.1);
    }

    /** <p>Set the distance the arm should move in mm</p>
     *  The typical distance is 230 mm of full travel.
     *  It takes around 0.2 power to get it moving. Springs on the end vary to amount as it extends.
     *  The PID is tuned as best as it can to compensate for many variables like the spring, friction, arm angles, ...
     *
     * @param _mm The mm out the arm should move to.
     */
    public void setPosition(double _mm){
        double pid = m_pid.calculate(getPosition(),_mm);
        move(pid);
    }
    public double getPosition(){
        return m_motor.getCurrentPosition() / k.FOREARM.Motor_CountsPmm;
    }

    /**
     * This is used during climbing to give the motor raw power from the thumbstick.
     * Limits are applied here
     * @param _speed The speed to move the motor.
     */
    public void move(double _speed){
        m_actualSpeed = _speed;
        m_speed = limitTravel(_speed);
        m_motor.set(limitTravel(_speed));
    }
    public double limitTravel(double _speed){
        if(_speed > 0){ // Extending and speed is positive
            if(getPosition() >= k.FOREARM.ExtendLimit){
                return 0;
            }else {
                return _speed;
            }
        }else {  // Retracting with negative speed
            if(getPosition() <= k.FOREARM.RetractLimit){
                return 0;
            }else {
                return _speed;
            }
        }
    }
    public double getSpeed(){
        return m_speed;
    }
    public double getActualSpeed(){
        return m_actualSpeed;
    }

}
