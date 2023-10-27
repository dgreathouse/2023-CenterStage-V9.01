package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/**
 *
 */
public class ArmGetTeamPropLocation extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;
    TeamPropLocation m_teamPropLocation;

    boolean isFinished = false;

    public ArmGetTeamPropLocation(CommandOpMode _opMode, ArmSubsystem _arm, TeamPropLocation _location) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_teamPropLocation = _location;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        double dis = m_arm.getTeamPropDistance();
        for (int i = 0; i < 4; i++) {
            dis += m_arm.getTeamPropDistance();
        }
        dis = dis / 5.0;

        if (m_teamPropLocation == TeamPropLocation.CENTER) {
            if (dis < 500) {
                GlobalData.TeamPropLocation = TeamPropLocation.CENTER;
            }
        } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
            if (dis < 500) {
                GlobalData.TeamPropLocation = TeamPropLocation.RIGHT;
            }
        }else if(m_teamPropLocation == TeamPropLocation.LEFT){
            if(GlobalData.TeamPropLocation == TeamPropLocation.NONE){
                GlobalData.TeamPropLocation = TeamPropLocation.LEFT;
            }
        }

        isFinished = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean _interrupted) {

    }
}
