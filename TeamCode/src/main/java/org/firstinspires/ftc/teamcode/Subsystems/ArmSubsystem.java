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

    /**
     * Set the arm, shoulder and Claw position with an enum value
     *
     * @param _pos
     */
    public void armGotoPosition(ArmPos _pos) {

    }
    public void armGotoPosition(){
        // Use the m_armAng and move the Shoulder, Claw and Forearm
        // Areas to deal with. Less than 0 degrees when straight.
        //   Claw needs to be parallel with the ground not the backdrop
        // Claw needs to be at 30 degrees always when greater than 0 degrees.

        // Set the shoulder
        setShoulderAngle(m_armAng);
        // Set the Forearm
        if(m_armAng < 0){
            setForearmPosition(0);
        }else {
            setForearmPosition(m_armAng * k.FOREARM.ExtentScale_mmPdeg);
        }

        // Set the Claw
        if(m_armAng < k.CLAW.ShoulderAngleAt30Deg){
            setClawAngle(m_armAng * k.CLAW.ShoulderAngleToFloorAng + k.CLAW.ShoulderAngleToFloorOffset);
        }else {
            setClawAngle(m_armAng * k.CLAW.ShoulderAngleToBackdropAng + k.CLAW.ShoulderAngleToBackdropOffset);
        }

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
                setArmAngle(0.0);
                break;
            case UP:
                setArmAngle(90);
                break;
            case STACK_3:
                setArmAngle(k.SHOULDER.RotateDownLimit + 8);
                break;
            case STACK_5:
                setArmAngle(k.SHOULDER.RotateDownLimit + 10);
                break;
            case FLOOR:
                setArmAngle(k.SHOULDER.RotateDownLimit);
                break;
            case ANGLE_30:
                setArmAngle(10);
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
    }
}
