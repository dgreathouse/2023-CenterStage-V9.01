package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

/**
 */
public class ArmGotoPosition extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;
    ArmPos m_pos;

    public ArmGotoPosition(CommandOpMode _opMode, ArmSubsystem _arm, ArmPos _pos) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_pos = _pos;
        addRequirements(m_arm);
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
