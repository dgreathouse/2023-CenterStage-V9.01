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
    double m_ang;

    public ArmGotoPosition(CommandOpMode _opMode, ArmSubsystem _arm, double _ang) {
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
