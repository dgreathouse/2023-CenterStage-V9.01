package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.opencv.core.Mat;

public class Shoulder {

    private  MotorEx m_motor;
    private CommandOpMode m_opMode;
    private PIDController rotPID;
    private ArmFeedforward armFeedforward;
    private double ks, kv, kCos;
    private double vel = 0.01;
    private double kfa;
    public Shoulder(CommandOpMode _opMode) {
        m_opMode = _opMode;
        initHardware();
    }

    public void initHardware(){
        m_motor = new MotorEx(m_opMode.hardwareMap, Hw.s_SH, MotorEx.GoBILDA.RPM_435);
        m_motor.setRunMode(Motor.RunMode.RawPower);  // Wish I had voltage mode
        m_motor.setInverted(false);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rotPID = new PIDController(0.06,0.01,0);
        rotPID.setTolerance(1);
        rotPID.setIntegrationBounds(-0.5,0.5);
        rotPID.reset();

        ks = 0.01;
        kv = 0.01;
        kCos = 0.01;
        vel = 0.01;
        kfa = .02; // Forearm Constant multiplier for arm extension.
        armFeedforward = new ArmFeedforward(0.001,0.05,0,0);

    }
    public void setAngle(double _ang){

        double ang = getAngle();
        ang -= 35;
        ang = Math.toRadians(ang);
        // Calculate the Arm Feedforward
        double cos = 0;
        // Forearm extends so Cos is not constant.
        if(ang > 0){
            cos = kCos * Math.cos(ang) * kfa;
        }else {
            cos = kCos * Math.cos(ang);
        }

        double rot = rotPID.calculate(getAngle(), _ang);
        double velSigned = vel * Math.signum(rot);
        double ff = ks * Math.signum(vel) + velSigned * cos + kv * vel;
        m_motor.set(rot + ff);
    }
    public double getAngle(){
       return  m_motor.getCurrentPosition() / k.SHOULDER.Motor_CountsPDeg;
    }

}
