package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.Interpolate;
import org.firstinspires.ftc.teamcode.Lib.k;

public class ArmSubsystem extends SubsystemBase {

    public double m_armAng = 0.0;
    public Rev2mDistanceSensor m_distanceSensor;
    private final CommandOpMode m_opMode;
    private Claw m_claw;
    private Shoulder m_shoulder;
    private Forearm m_forearm;

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
        m_shoulder = new Shoulder(m_opMode, this);
        m_forearm = new Forearm(m_opMode);
        m_distanceSensor = m_opMode.hardwareMap.get(Rev2mDistanceSensor.class, Hw.s_distance);
    }
    public double getForearmPositionFromAngle(){
        // 0 between 0 and 35
        // 0 to 230 from 35 to 85
        double sa = m_shoulder.getAngle();
        if(sa < getArmSetAngle(ArmPos.STRAIGHT)) {
            return 0;
        }else if(sa > getArmSetAngle(ArmPos.BACKDROPUPLIMIT)){
            return k.FOREARM.ExtendLimit;
        }else {
            return k.FOREARM.ExtendLimit/(getArmSetAngle(ArmPos.BACKDROPUPLIMIT) - getArmSetAngle(ArmPos.STRAIGHT)) * (sa - getArmSetAngle(ArmPos.STRAIGHT));
        }

    }

    public void armGotoPosition() {
        // Set the shoulder
        setShoulderAngle(m_armAng);
        //setShouldTestPower(GlobalData.ShoulderTestPower);
        // Set the Forearm

        setForearmPosition(getForearmPositionFromAngle());
         //m_forearm.move(GlobalData.ForearmTestPower);

        // Set the Claw
        //setClawTestPos(GlobalData.ClawAngTestPos);
       // setClawAngle(Interpolate.getY(k.ARM.ShoulderAngles, k.ARM.ClawAngles, m_armAng));
    }

    public void setShoulderAngle(double _angle) {
        m_shoulder.setAngle(_angle);
    }

    public void setForearmPosition(double _mm) {
        m_opMode.telemetry.addData("Forearm mm", _mm);
        m_forearm.setPosition(_mm);
    }

    public void setClawAngle(double _ang) {
        m_claw.setClawRotateAngle(_ang);
    }

    public void armForearmMove(double _speed) {
        // Limits are applied in the forearm
        m_forearm.move(_speed);
    }
    // Team Prop functions

    public double getTeamPropDistance() {

        return m_distanceSensor.getDistance(DistanceUnit.MM);
    }

    // CLAW functions
    public void setClawGripAngle(double _angle) {
        m_claw.setClawGripAngle(_angle);
    }

    public double getClawCloseAngle() {
        if (GlobalData.TeamNumber == 22291) {
            return k.CLAW.CloseAngle_22291;
        }
        return k.CLAW.CloseAngle_14623;
    }

    public double getClawOpenAngle() {
        if (GlobalData.TeamNumber == 22291) {
            return k.CLAW.OpenAngle_22291;
        }
        return k.CLAW.OpenAngle_14623;
    }

    public double getClawReleaseLowerAngle() {
        if (GlobalData.TeamNumber == 22291) {
            return k.CLAW.OpenLowerAngle_22291;
        }
        return k.CLAW.OpenLowerAngle_14623;
    }

    public double getClawReleaseUpperAngle() {
        if (GlobalData.TeamNumber == 22291) {
            return k.CLAW.OpenUpperAngle_22291;
        }
        return k.CLAW.OpenUpperAngle_14623;
    }

    public void setArmPosition(ArmPos _armPos) {
        switch (_armPos) {
            case STRAIGHT:
                setArmAngle(35);
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

    public void setArmAngle(double _angle) {
        m_armAng = _angle;
    }

    public double getArmSetAngle(ArmPos _armPos) {
        if (_armPos == ArmPos.STRAIGHT) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleStraight_22291;
            }
            return k.SHOULDER.AngleStraight_14623;
        }
        if (_armPos == ArmPos.STACK_5) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleStack_5_22291;
            }
            return k.SHOULDER.AngleStack_5_14623;
        }
        if (_armPos == ArmPos.FLOOR) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleFloor_22291;
            }
            return k.SHOULDER.AngleFloor_14623;
        }
        if (_armPos == ArmPos.STACK_3) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleStack_3_22291;
            }
            return k.SHOULDER.AngleStack_3_14623;
        }
        if (_armPos == ArmPos.BACKDROPUPLIMIT) {
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleBackdropUpLimit_22291;
            }
            return k.SHOULDER.AngleBackdropUpLimit_14623;
        }
        if(_armPos == ArmPos.VERTICAL){
            if (GlobalData.TeamNumber == 22291) {
                return k.SHOULDER.AngleVertical_22291;
            }
            return k.SHOULDER.AngleVertical_14623;
        }
        return k.SHOULDER.AngleStraight_14623;

    }

    public void setShouldTestPower(double _pwr) {
        m_shoulder.setPower(_pwr);
    }

    public void setClawTestPos(double _pos) {
        m_claw.setClawRotateAngle(_pos);
    }

    @Override
    public void periodic() {
        m_opMode.telemetry.addData("TeamPropDis", getTeamPropDistance());
        m_opMode.telemetry.addData("Shoulder Angle", m_shoulder.getAngle());
        m_opMode.telemetry.addData("Claw Angle", m_claw.getClawRotateAngle());
        m_opMode.telemetry.addData("Claw Grip Angle", m_claw.getClawGripAngle());
        m_opMode.telemetry.addData("Forearm Distance", m_forearm.getPosition());
        m_opMode.telemetry.addData("Shoulder Test Power", GlobalData.ShoulderTestPower);
        m_opMode.telemetry.addData("Forearm Test Power", GlobalData.ForearmTestPower);
        m_opMode.telemetry.addData("Requested Arm Angle", m_armAng);

    }
}
