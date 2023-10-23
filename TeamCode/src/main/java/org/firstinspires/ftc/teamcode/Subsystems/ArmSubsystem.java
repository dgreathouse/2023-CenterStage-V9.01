package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.Interpolate;
import org.firstinspires.ftc.teamcode.Lib.k;

public class ArmSubsystem extends SubsystemBase {

    private CommandOpMode m_opMode;
    private Claw m_claw;
    private Shoulder m_shoulder;
    private Forearm m_forearm;
    private ServoEx m_teamPropServo;

    public double m_armAng = 0.0;
    public Rev2mDistanceSensor m_distanceSensor;

    public ArmSubsystem(CommandOpMode _opMode) {
        m_opMode = _opMode;

        initHardware();
    }

    /**
     * Initialize the hardware for this module.
     * This is called during the constructor
     */
    private void initHardware() {
        m_claw = new Claw(m_opMode);
        m_shoulder = new Shoulder(m_opMode);
        m_forearm = new Forearm(m_opMode);
        m_teamPropServo = new SimpleServo(m_opMode.hardwareMap, Hw.s_distanceServo, 0, 300, AngleUnit.DEGREES);
        m_distanceSensor = m_opMode.hardwareMap.get(Rev2mDistanceSensor.class, Hw.s_distance);
    }
    public void armGotoPosition(){
        // Set the shoulder
        setShoulderAngle(m_armAng);
        // Set the Forearm
        if(m_armAng < -k.SHOULDER.ThumbRotateDownLimit){
            setForearmPosition(0);
        }else {
            setForearmPosition((m_armAng+k.SHOULDER.ThumbRotateDownLimit) * k.FOREARM.ExtentScale_mmPdeg);
        }
        // Set the Claw
        setClawAngle(Interpolate.getY(k.ARM.ShoulderAngles,k.ARM.ClawAngles,m_armAng));
    }
    public void setShoulderAngle(double _angle){
        m_shoulder.setAngle((int) _angle);
    }
    public void setForearmPosition(double _mm){
        m_forearm.setPosition((int)_mm);
    }
    public void setClawAngle(double _ang){
        m_claw.setClawRotateAngle((int)_ang,0.4);
    }
    public void armForearmMove(double _speed){
        // Limits are applied in the forearm
        m_forearm.move(_speed);
    }
    // Team Prop functions
    public void setTeamPropServo(int _pos){
        m_teamPropServo.setPosition(_pos);
    }
    public double getTeamPropDistance(){
        return m_distanceSensor.getDistance(DistanceUnit.MM);
    }
    // CLAW functions
    public void setClawGripAngle(double _angle){
        m_claw.setClawGripAngle(_angle);
    }
    public void setArmPosition(ArmPos _armPos){
        switch (_armPos){
            case STRAIGHT:
                setArmAngle(Math.abs(k.SHOULDER.DownLimit - k.SHOULDER.ThumbRotateDownLimit));
                break;
             case STACK_3:
                setArmAngle(8);
                break;
            case STACK_5:
                setArmAngle(10);
                break;
            case FLOOR:
                setArmAngle(0);
                break;
            case NONE:
                break;
        }
    }
    public void setArmAngle(double _angle){
        m_armAng = _angle;
    }
    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Claw Angle", m_claw.getClawRotateAngle());
        m_opMode.telemetry.addData("m_armAng", m_armAng);
    }
}
