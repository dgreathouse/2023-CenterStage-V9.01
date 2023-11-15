package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;

/**
 */
public class ArmAutoGotoPosition extends CommandBase {
    CommandOpMode m_opMode;
    AutoArmSubsystem m_arm;
    double m_shoulderAngle;
    double m_forearmPosition;
    double m_clawAngle;
    public ArmAutoGotoPosition(CommandOpMode _opMode, AutoArmSubsystem _arm, double _shoulderAngle, double _clawAngle, double _forearmPosition) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_shoulderAngle = _shoulderAngle;
        m_forearmPosition = _forearmPosition;
        m_clawAngle = _clawAngle;
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){
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
