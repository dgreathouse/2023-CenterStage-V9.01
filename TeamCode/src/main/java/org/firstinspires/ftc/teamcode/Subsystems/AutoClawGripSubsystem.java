package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;

public class AutoClawGripSubsystem extends SubsystemBase {
    ServoEx m_right;
    private CommandOpMode m_opMode;
    double m_gripAngle = getClawCloseAngle();
    double x = 1;
    public AutoClawGripSubsystem(CommandOpMode _opMode){
        m_opMode = _opMode;
        initHardware();
    }
    private void initHardware() {
        m_right = new SimpleServo(m_opMode.hardwareMap,Hw.ClawServoRotate, 0, 300, AngleUnit.DEGREES);
    }
    public void setClawGripAngle(double _angle){
        double angle = _angle / 300.0;
        m_gripAngle = _angle;
        m_right.setPosition(angle);
    }
    public double getClawGripAngle(){
        return m_gripAngle;
    }
    public void setClawCloseAngle(){
        double angle = GlobalData.TeamNumber == 22291 ? k.CLAW.CloseAngle_22291: k.CLAW.CloseAngle_14623;
        angle /= 300.0;
        m_gripAngle = angle;
        m_right.setPosition(angle);
    }
    public void setClawOpenAngle(){
        double angle = GlobalData.TeamNumber == 22291 ? k.CLAW.OpenAngle_22291: k.CLAW.OpenAngle_14623;
        angle /= 300.0;
        m_gripAngle = angle;
        m_right.setPosition(angle);
    }
    public void setClawReleaseLowerAngle(){
        double angle = GlobalData.TeamNumber == 22291 ? k.CLAW.OpenLowerAngle_22291: k.CLAW.OpenLowerAngle_14623;
        angle /= 300.0;
        m_gripAngle = angle;
        m_right.setPosition(angle);
    }
    public void setClawReleaseUpperAngle(){
        double angle = GlobalData.TeamNumber == 22291 ? k.CLAW.OpenUpperAngle_22291: k.CLAW.OpenUpperAngle_14623;
        angle /= 300.0;
        m_gripAngle = angle;
        m_right.setPosition(angle);
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
    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Claw Grip Angle", "%3.3f", getClawGripAngle());
        setClawGripAngle(m_gripAngle);
    }
}
