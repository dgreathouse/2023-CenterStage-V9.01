package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.opencv.core.Mat;

public class Shoulder {

    private  MotorEx m_motor;
    private CommandOpMode m_opMode;
    private PIDController rotPID;
    private ArmFeedforward armFeedforward;
    private ArmSubsystem m_arm;
    private double ks, kv, kCos;
    private double vel = 0.01;
    private double kfa, kfaMax;
    public Shoulder(CommandOpMode _opMode, ArmSubsystem _arm) {
        m_opMode = _opMode;
        m_arm = _arm;
        initHardware();
    }

    public void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.s_SH, MotorEx.GoBILDA.RPM_435);
        m_motor.setRunMode(Motor.RunMode.RawPower);  // Wish I had voltage mode
        m_motor.setInverted(false);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rotPID = new PIDController(0.0075,0.025,0);
        rotPID.setTolerance(1);
        rotPID.setIntegrationBounds(-5,5);
        rotPID.reset();

        ks = 0.0;
        kv = 0.0;
        kCos = 0.2;
        vel = 0.0;
        kfa = 0.2; // Forearm Constant multiplier for arm extension.
        kfaMax = 0.45;
        //armFeedforward = new ArmFeedforward(ks,kCos,kv,0);

    }
    public void setAngle(double _ang){

        double Cosang = getAngle();
        Cosang -= m_arm.getArmSetAngle(ArmPos.STRAIGHT);
        Cosang = Math.toRadians(Cosang);
        double ShAngRad = Math.toRadians(_ang);
        double kfaScale = (kfaMax - kfa) / Math.toRadians(m_arm.getArmSetAngle(ArmPos.BACKDROPUPLIMIT) - m_arm.getArmSetAngle(ArmPos.STRAIGHT));
        // Calculate the Arm Feedforward
        double cos = Math.cos(Cosang) * kfa;
        // Between the area where the arm extends and changes the cos value
        if(Cosang > 0 && Cosang < Math.toRadians(m_arm.getArmSetAngle(ArmPos.BACKDROPUPLIMIT))){
            cos = cos + Cosang * kfaScale;
        }else if(Cosang > m_arm.getArmSetAngle(ArmPos.BACKDROPUPLIMIT)){
            cos = Math.cos(Cosang) * kfa * 0.8;
        }

        double rot = rotPID.calculate(getAngle(), _ang);

        m_motor.set(cos + rot);
    }
    public void setPower(double _pwr){
        m_motor.set(_pwr);
    }
    public double getAngle(){
       return  m_motor.getCurrentPosition() / k.SHOULDER.Motor_CountsPDeg;
    }

}
