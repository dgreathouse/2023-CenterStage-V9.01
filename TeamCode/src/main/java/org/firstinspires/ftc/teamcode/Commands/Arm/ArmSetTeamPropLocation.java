package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

import java.util.concurrent.TimeUnit;

/**
 */
public class ArmSetTeamPropLocation extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;
    double pos = 0;
    boolean isFinished = false;
    public ArmSetTeamPropLocation(CommandOpMode _opMode, ArmSubsystem _arm, double _pos) {
        m_opMode = _opMode;
        m_arm = _arm;
        pos = _pos;
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){


    }
    @Override
    public void execute(){
        m_arm.setTeamPropServo(pos);
        m_opMode.telemetry.addData("DisSensor", m_arm.getTeamPropDistance());

        isFinished = true;
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }
    @Override
    public void end(boolean _interrupted){

    }
}
