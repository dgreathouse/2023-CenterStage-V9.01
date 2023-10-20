package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/**
 */
public class ArmRotateFingers extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;
    double m_angle;

    public ArmRotateFingers(CommandOpMode _opMode, ArmSubsystem _arm, double _angle) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_angle = _angle;

    }

    @Override
    public void initialize(){
        m_arm.setClawGripAngle(m_angle);
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
