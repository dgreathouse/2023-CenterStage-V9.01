package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/**
 */
public class ArmGetTeamPropLocation extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;


    public ArmGetTeamPropLocation(CommandOpMode _opMode, ArmSubsystem _arm) {
        m_opMode = _opMode;
        m_arm = _arm;


    }

    @Override
    public void initialize(){

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
