package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;

/** Arm Auto Default Command

 */
public class AutoArmDefaultCommand extends CommandBase {
    AutoArmSubsystem m_arm;
    CommandOpMode m_opMode;
    int x = 0;

    public AutoArmDefaultCommand(CommandOpMode _opMode, AutoArmSubsystem _arm){
        m_opMode = _opMode;
        m_arm = _arm;

        addRequirements(m_arm);
    }
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(GlobalData.ArmAutoEnable){
            m_arm.armGotoPosition();

        }
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    @Override
    public void end(boolean _interrupted){

    }
}
