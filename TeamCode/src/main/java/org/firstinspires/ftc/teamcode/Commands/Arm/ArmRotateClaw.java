package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/**
 */
public class ArmRotateClaw extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;
    ArmPos m_pos;

    public ArmRotateClaw(CommandOpMode _opMode, ArmSubsystem _arm, ArmPos _pos) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_pos = _pos;

    }

    @Override
    public void initialize(){
        m_arm.armGotoPosition(m_pos);
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
