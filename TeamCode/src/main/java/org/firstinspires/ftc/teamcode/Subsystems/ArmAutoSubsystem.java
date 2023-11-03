package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.Interpolate;
import org.firstinspires.ftc.teamcode.Lib.k;

public class ArmAutoSubsystem extends SubsystemBase {
    private final CommandOpMode m_opMode;
    private Claw m_claw;
    private Forearm m_forearm;
    private Shoulder m_shoulder;
    public Rev2mDistanceSensor m_distanceSensor;
    public ArmData m_armData;
    public double m_armAng = 0.0;
    private double m_IClawAngle = 0;
    public ArmAutoSubsystem(CommandOpMode _opMode) {
        m_opMode = _opMode;
        initHardware();
    }

    /**
     * Initialize the hardware for this module.
     * This is called during the constructor
     */
    private void initHardware() {
        m_claw = new Claw(m_opMode);
        m_forearm = new Forearm(m_opMode);
        m_armData = new ArmData();
        m_shoulder = new Shoulder(m_opMode, m_armData);
        m_distanceSensor = m_opMode.hardwareMap.get(Rev2mDistanceSensor.class, Hw.DistanceSensor);
    }

    /** Main function for the arm. This will move the shoulder, forearm and claw to the correct angles.
     * The method setArmAngle(double _angle) or setArmAngle(ArmPos _armPos) must be called before
     * this method to set the global variable <b>m_armAng.</b>
     * <p>Since this method is called constantly the arm will go to whatever angle the m_armAng variable is set to
     * </p>
     *
     */
    public void armGotoPosition() {
        // Set the shoulder
        setShoulderAngle(m_armAng);

        // Set the Forearm
        setForearmPosition(getForearmPositionFromAngle());

        // Set the Claw
        m_IClawAngle = Interpolate.getY(k.ARM.ShoulderAngles, k.ARM.ClawAngles, m_shoulder.getAngle());
        setClawAngle(m_IClawAngle);
    }
    private double getForearmPositionFromAngle(){
        // 0 between 0 and 35
        // 0 to 230 from 35 to 85
        double sa = m_shoulder.getAngle();
        if(sa < m_armData.getArmSetAngle(ArmPos.STRAIGHT)) {
            return 0;
        }else if(sa > m_armData.getArmSetAngle(ArmPos.BACKDROPUPLIMIT)){
            return k.FOREARM.ExtendLimit;
        }else {
            return k.FOREARM.ExtendLimit/(m_armData.getArmSetAngle(ArmPos.BACKDROPUPLIMIT) - m_armData.getArmSetAngle(ArmPos.STRAIGHT)) * (sa - m_armData.getArmSetAngle(ArmPos.STRAIGHT));
        }
    }
    private void setForearmPosition(double _mm) {
        m_opMode.telemetry.addData("Forearm mm", _mm);
        m_forearm.setPosition(_mm);
    }
    private void setShoulderAngle(double _angle) {
        m_shoulder.setAngle(_angle);
    }


    // Team Prop functions
    public double getTeamPropDistance() {
        return m_distanceSensor.getDistance(DistanceUnit.MM);
    }

    // CLAW functions
    private void setClawAngle(double _ang) {
        m_claw.setClawRotateAngle(_ang);
    }

    public void setArmAngle(double _angle) {
        m_armAng = _angle;
    }

    @Override
    public void periodic() {
        armGotoPosition();
        m_opMode.telemetry.addData("Team Prop Distance", "%3.3f", getTeamPropDistance());
        m_opMode.telemetry.addData("Team Prop Location", GlobalData.TeamPropLocation);
        m_opMode.telemetry.addData("Shoulder Angle", "%3.3f", m_shoulder.getAngle());
        m_opMode.telemetry.addData("Shoulder Power", "%3.3f", m_shoulder.getPower());
        m_opMode.telemetry.addData("Shoulder Angle Requested", "%3.3f", m_armAng);
        m_opMode.telemetry.addData("Shoulder Up Vel Limit", "%3.3f",k.SHOULDER.RotationPID_Max);
        m_opMode.telemetry.addData("Shoulder Down Vel Limit", "%3.3f",k.SHOULDER.RotationPID_Min);
        m_opMode.telemetry.addData("Claw Angle", "%3.3f", m_claw.getClawRotateAngle());
        m_opMode.telemetry.addData("Claw Requested Angle", "%3.3f", m_IClawAngle);


        m_opMode.telemetry.addData("Forearm Distance", "%3.3f", m_forearm.getPosition());
        m_opMode.telemetry.addData("Forearm Speed", "%3.3f", m_forearm.getSpeed());
        m_opMode.telemetry.addData("Forearm Actual Speed", "%3.3f", m_forearm.getActualSpeed());

   //     m_opMode.telemetry.addData("Shoulder Test Power", GlobalData.ShoulderTestPower);
   //     m_opMode.telemetry.addData("Forearm Test Power", GlobalData.ForearmTestPower);


    }
}
