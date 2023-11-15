package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;

public class Shoulder {

    private  MotorEx m_motor;
    private CommandOpMode m_opMode;
    private PIDController rotPID;
    private ArmData m_arm; // Used here to get the limits from the arm.
    private Forearm m_forearm;
    private double ks, kv, kCos;
    private double vel = 0.01;
    private double kfa, kfaMax;
    private double m_shoulderPower = 0;
    private double motor_v = 0.0;
    private double kMaxVel = 300;
    private double GravityBoost = 0;
    double m_kfaScale = (kfaMax - kfa) / (k.FOREARM.ExtendLimit - k.FOREARM.RetractLimit);
    public Shoulder(CommandOpMode _opMode, ArmData _arm, Forearm _forearm) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_forearm = _forearm;
        initHardware();
    }

    private void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.ShoulderMotor, MotorEx.GoBILDA.RPM_435);
        m_motor.setRunMode(Motor.RunMode.RawPower);  // Wish I had voltage mode
        m_motor.setInverted(false);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.resetEncoder();
        rotPID = new PIDController(.62,0.15,0.0);
        rotPID.setTolerance(.01);
        rotPID.setIntegrationBounds(-2,2);
        rotPID.reset();

        ks = 0.0;
        kv = 0.002;
        kCos = 0.2;
        vel = 0.0;
        kfa = kCos; // Forearm Constant multiplier for arm extension.
        kfaMax = 0.45;
    }


    public void setAngle(double _ang){
        // Convert the angle to radians for Cos function
        double CurrentAngleOffsetRad = Math.toRadians(getAngle() - m_arm.getArmSetAngle(ArmPos.STRAIGHT));
        double RequestedAngleOffsetRad = Math.toRadians(_ang - m_arm.getArmSetAngle(ArmPos.STRAIGHT));
        motor_v = m_motor.getCorrectedVelocity();
        motor_v = MathUtils.clamp(motor_v, 0, kMaxVel);
        double cos = Math.cos(CurrentAngleOffsetRad) * kfa;
        // Add a scale based on the actual forearm position
        cos = cos + m_forearm.getPosition() * Math.signum(cos) * m_kfaScale;
        double error_p = RequestedAngleOffsetRad - CurrentAngleOffsetRad;
        // The PID is basically the velocity the arm will rotate at.
        double rot = rotPID.calculate(CurrentAngleOffsetRad, RequestedAngleOffsetRad);
        // Clamp the value of rotation to a minimum to slow the down speed and positive up value to raise the arm with more power.
        rot = MathUtils.clamp(rot,k.SHOULDER.RotationPID_Min,2.5);

        if(rot > 0) { // rot is positive when trying to raise the arm
            GravityBoost = (kMaxVel - motor_v) * kv * error_p;
            rot = rot + GravityBoost;
        }
        // Set a module level variable to allow for telemetry
        m_shoulderPower = cos + rot;
        if(getAngle() < 10 && _ang < 1.0){
            m_motor.set(0.0);
        }else {
            m_motor.set(m_shoulderPower);
        }


    }
    public double getGravityBoost(){
        return GravityBoost;
    }
    public double getVelocity(){
       return m_motor.getCorrectedVelocity();
    }
    public double getAngle(){
       return  m_motor.getCurrentPosition() / k.SHOULDER.Motor_CountsPDeg;
    }
    public double getPower(){
        return m_shoulderPower;
    }
}
