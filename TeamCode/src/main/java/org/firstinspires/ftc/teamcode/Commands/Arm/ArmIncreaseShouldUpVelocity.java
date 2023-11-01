package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/**
 */
public class ArmIncreaseShouldUpVelocity extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;
    double m_pwr;

    public ArmIncreaseShouldUpVelocity(CommandOpMode _opMode, ArmSubsystem _arm, double _pwr) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_pwr = _pwr;
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){
        k.SHOULDER.RotationPID_Max += m_pwr;
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
