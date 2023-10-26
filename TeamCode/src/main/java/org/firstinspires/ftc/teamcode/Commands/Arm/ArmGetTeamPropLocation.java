package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

import java.util.concurrent.TimeUnit;

/**
 */
public class ArmGetTeamPropLocation extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;
    TeamPropLocation m_teamPropLocation;
    Timing.Timer m_servoTiming;
    boolean isFinished = false;
    public ArmGetTeamPropLocation(CommandOpMode _opMode, ArmSubsystem _arm, TeamPropLocation _location) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_teamPropLocation = _location;
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){
       m_servoTiming = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
    }

    @Override
    public void execute(){
        double dis = m_arm.getTeamPropDistance();
        for(int i = 0; i < 4; i++){
            dis += m_arm.getTeamPropDistance();
        }
        dis = dis/5.0;
        if(m_teamPropLocation == TeamPropLocation.CENTER){
            if(dis < 500){
                GlobalData.TeamPropLocation = TeamPropLocation.CENTER;
            }
        }else if(m_teamPropLocation == TeamPropLocation.RIGHT){
            if(dis < 500){
                GlobalData.TeamPropLocation = TeamPropLocation.RIGHT;
            }
        }else if(m_teamPropLocation == TeamPropLocation.LEFT){
            if(dis < 500){
                GlobalData.TeamPropLocation = TeamPropLocation.LEFT;
            }
        }else {
            GlobalData.TeamPropLocation = TeamPropLocation.CENTER;
        }
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
