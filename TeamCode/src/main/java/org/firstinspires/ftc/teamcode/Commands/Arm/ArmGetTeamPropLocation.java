package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.ArmAutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/**
 *
 */
public class ArmGetTeamPropLocation extends CommandBase {
    CommandOpMode m_opMode;
    ArmAutoSubsystem m_arm;
    TeamPropLocation m_teamPropLocation;

    boolean isFinished = false;

    public ArmGetTeamPropLocation(CommandOpMode _opMode, ArmAutoSubsystem _arm, TeamPropLocation _location) {
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
        if (GlobalData.FieldLocation == AutoFieldLocation.BACKDROP) {
            if (GlobalData.TeamColor == TeamColor.RED) {
                if (m_teamPropLocation == TeamPropLocation.CENTER) {
                    if (dis < 500) {
                        GlobalData.TeamPropLocation = TeamPropLocation.CENTER;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
                    if (dis < 500) {
                        GlobalData.TeamPropLocation = TeamPropLocation.RIGHT;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.LEFT) {
                    if (GlobalData.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.TeamPropLocation = TeamPropLocation.LEFT;
                    }
                }
            } else { //Blue
                if (m_teamPropLocation == TeamPropLocation.CENTER) {
                    if (dis < 500) {
                        GlobalData.TeamPropLocation = TeamPropLocation.CENTER;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.LEFT) {
                    if (dis < 500) {
                        GlobalData.TeamPropLocation = TeamPropLocation.LEFT;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
                    if (GlobalData.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.TeamPropLocation = TeamPropLocation.RIGHT;
                    }
                }
            }
        } else { // WING
            if (GlobalData.TeamColor == TeamColor.RED) {
                if (m_teamPropLocation == TeamPropLocation.CENTER) {
                    if (dis < 500) {
                        GlobalData.TeamPropLocation = TeamPropLocation.CENTER;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.LEFT) {
                    if (dis < 500) {
                        GlobalData.TeamPropLocation = TeamPropLocation.LEFT;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
                    if (GlobalData.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.TeamPropLocation = TeamPropLocation.RIGHT;
                    }
                }
            } else { //Blue
                if (m_teamPropLocation == TeamPropLocation.CENTER) {
                    if (dis < 500) {
                        GlobalData.TeamPropLocation = TeamPropLocation.CENTER;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
                    if (dis < 500) {
                        GlobalData.TeamPropLocation = TeamPropLocation.RIGHT;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.LEFT) {
                    if (GlobalData.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.TeamPropLocation = TeamPropLocation.LEFT;
                    }
                }

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
