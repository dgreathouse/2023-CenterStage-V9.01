package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation_enum;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;

/**
 *
 */
public class ArmGetTeamPropLocation extends CommandBase {
    CommandOpMode m_opMode;
    AutoArmSubsystem m_arm;
    TeamPropLocation m_teamPropLocation;

    boolean isFinished = false;

    public ArmGetTeamPropLocation(CommandOpMode _opMode, AutoArmSubsystem _arm, TeamPropLocation _location) {
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
        if (GlobalData.MATCH.AutoFieldLocation == AutoFieldLocation_enum.BACKDROP) {
            if (GlobalData.MATCH.AutoTeamColor == TeamColor.RED) {
                if (m_teamPropLocation == TeamPropLocation.CENTER) {
                    if (dis < 500) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
                        GlobalData.tagOfInterest = 5;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
                    if (dis < 500) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                        GlobalData.tagOfInterest = 6;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.LEFT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                        GlobalData.tagOfInterest = 4;
                    }
                }
            } else { //Blue
                if (m_teamPropLocation == TeamPropLocation.CENTER) {
                    if (dis < 500) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
                        GlobalData.tagOfInterest = 2;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.LEFT) {
                    if (dis < 500) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                        GlobalData.tagOfInterest = 1;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                        GlobalData.tagOfInterest = 3;
                    }
                }
            }
        } else { // WING
            if (GlobalData.MATCH.AutoTeamColor == TeamColor.RED) {
                if (m_teamPropLocation == TeamPropLocation.CENTER) {
                    if (dis < 500) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
                        GlobalData.tagOfInterest = 5;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.LEFT) {
                    if (dis < 500) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                        GlobalData.tagOfInterest = 4;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                        GlobalData.tagOfInterest = 6;
                    }
                }
            } else { //Blue
                if (m_teamPropLocation == TeamPropLocation.CENTER) {
                    if (dis < 500) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
                        GlobalData.tagOfInterest = 2;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.RIGHT) {
                    if (dis < 500) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                        GlobalData.tagOfInterest = 3;
                    }
                } else if (m_teamPropLocation == TeamPropLocation.LEFT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                        GlobalData.tagOfInterest = 1;
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
