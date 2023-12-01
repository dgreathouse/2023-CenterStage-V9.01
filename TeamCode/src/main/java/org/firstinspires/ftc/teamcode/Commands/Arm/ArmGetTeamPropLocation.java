package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation_enum;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;

/**
 *
 */
public class ArmGetTeamPropLocation extends CommandBase {
    CommandOpMode m_opMode;
    AutoArmSubsystem m_arm;
    TeamPropLocation m_teamPropCheckedLocation;

    boolean isFinished = false;

    public ArmGetTeamPropLocation(CommandOpMode _opMode, AutoArmSubsystem _arm, TeamPropLocation _location) {
        m_opMode = _opMode;
        m_arm = _arm;
        m_teamPropCheckedLocation = _location;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        double dis = m_arm.getTeamPropDistance();
        for (int i = 0; i < 4; i++) {
            dis += m_arm.getTeamPropDistance();
        }
        dis = dis / 5.0;
        if (GlobalData.MATCH.AutoFieldLocation == AutoFieldLocation_enum.BACKDROP) { // Backdrop
            if (GlobalData.MATCH.AutoTeamColor == TeamColor.RED) { //Red
                if (m_teamPropCheckedLocation == TeamPropLocation.CENTER) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
                        GlobalData.tagOfInterest = 5;
                    }
                } else if (m_teamPropCheckedLocation == TeamPropLocation.RIGHT) {
                    if (dis <  k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                        GlobalData.tagOfInterest = 6;
                    }
                } else if (m_teamPropCheckedLocation == TeamPropLocation.LEFT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                        GlobalData.tagOfInterest = 4;
                    }
                }
            } else { //Blue
                if (m_teamPropCheckedLocation == TeamPropLocation.CENTER) {
                    if (dis <  k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
                        GlobalData.tagOfInterest = 2;
                    }
                } else if (m_teamPropCheckedLocation == TeamPropLocation.LEFT) {
                    if (dis <  k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                        GlobalData.tagOfInterest = 1;
                    }
                } else if (m_teamPropCheckedLocation == TeamPropLocation.RIGHT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                        GlobalData.tagOfInterest = 3;
                    }
                }
            }
        } else { // WING
            if (GlobalData.MATCH.AutoTeamColor == TeamColor.RED) { //Red
                if (m_teamPropCheckedLocation == TeamPropLocation.CENTER) {
                    if (dis <  k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
                        GlobalData.tagOfInterest = 5;
                    }
                } else if (m_teamPropCheckedLocation == TeamPropLocation.LEFT) {
                    if (dis <  k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                        GlobalData.tagOfInterest = 4;
                    }
                } else if (m_teamPropCheckedLocation == TeamPropLocation.RIGHT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                        GlobalData.tagOfInterest = 6;
                    }
                }
            } else { //Blue
                if (m_teamPropCheckedLocation == TeamPropLocation.CENTER) {
                    if (dis <  k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
                        GlobalData.tagOfInterest = 2;
                    }
                } else if (m_teamPropCheckedLocation == TeamPropLocation.RIGHT) {
                    if (dis <  k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                        GlobalData.tagOfInterest = 3;
                    }
                } else if (m_teamPropCheckedLocation == TeamPropLocation.LEFT) {
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
