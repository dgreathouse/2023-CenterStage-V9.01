package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmAutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/**
 */
public class ArmAutoGotoPosition extends CommandBase {
    CommandOpMode m_opMode;
    ArmAutoSubsystem m_arm;
    double m_ang;

    public ArmAutoGotoPosition(CommandOpMode _opMode, ArmAutoSubsystem _arm, double _ang) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_ang = _ang;
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){
        m_arm.setArmAngle(m_ang);
    }
    @Override
    public void execute(){

    }
    @Override
    public boolean isFinished(){
        return true;
    }
    @Override
    public void end(boolean _interrupted){

    }
}
