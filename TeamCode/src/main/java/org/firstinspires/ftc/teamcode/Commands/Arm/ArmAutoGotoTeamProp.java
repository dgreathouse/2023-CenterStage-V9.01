package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;

/**
 */
public class ArmAutoGotoTeamProp extends CommandBase {
    CommandOpMode m_opMode;
    AutoArmSubsystem m_arm;
    double m_shoulderAngle;
    double m_forearmPosition;
    double m_clawAngle;
    public ArmAutoGotoTeamProp(CommandOpMode _opMode, AutoArmSubsystem _arm) {
        m_opMode = _opMode;
        m_arm = _arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){
        switch (GlobalData.MATCH.TeamPropLocation) {
            case CENTER:
            case NONE:
                m_shoulderAngle = 35;
                m_clawAngle = -10;
                m_forearmPosition = 100;
                break;
            case LEFT:
            case RIGHT:
                m_shoulderAngle = 35;
                m_clawAngle = -10;
                m_forearmPosition = 150;
                break;

        }
        m_arm.setArmData(m_shoulderAngle, m_clawAngle, m_forearmPosition);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    @Override
    public void end(boolean _interrupted){

    }
}
