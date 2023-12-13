package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation_enum;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Lib.k;

public class AutoArmSubsystem extends SubsystemBase {
    private final CommandOpMode m_opMode;
    private Claw m_claw;
    private Forearm m_forearm;
    private Shoulder m_shoulder;
    private Rev2mDistanceSensor m_distanceSensor;
    private double m_requestedArmAng = 0.0;
    private double m_requestedForearmPosition = 0.0;
    private double m_requestedClawAngle = 0;

    public AutoArmSubsystem(CommandOpMode _opMode) {
        m_opMode = _opMode;
        initHardware();
    }

    /**
     * Initialize the hardware for this module.
     * This is called during the constructor
     */
    private void initHardware() {
        m_claw = new Claw(m_opMode);
        m_forearm = new Forearm(m_opMode);
        ArmData m_armData = new ArmData();
        m_shoulder = new Shoulder(m_opMode, m_armData, m_forearm);
        m_distanceSensor = m_opMode.hardwareMap.get(Rev2mDistanceSensor.class, Hw.DistanceSensor);
    }

    /**
     * Main function for the arm.
     */
    public void armGotoPosition() {
        // Set the shoulder
        setShoulderAngle(m_requestedArmAng);

        // Set the Forearm
        setForearmPosition(m_requestedForearmPosition);

        // Set the Claw
        setClawAngle(m_requestedClawAngle);
    }

    private void setShoulderAngle(double _angle) {
        m_shoulder.setAngle(_angle);
    }

    private void setForearmPosition(double _mm) {
        m_opMode.telemetry.addData("Forearm mm", _mm);
        m_forearm.setPosition(_mm);
    }

    private void setClawAngle(double _ang) {
        m_claw.setClawRotateAngle(_ang);
    }

    public double getTeamPropDistance() {
        return m_distanceSensor.getDistance(DistanceUnit.MM);
    }

    public void setArmData(double _shoulderAngle, double _clawAngle, double _forearmPosition) {
        m_requestedArmAng = _shoulderAngle;
        m_requestedForearmPosition = _forearmPosition;
        m_requestedClawAngle = _clawAngle;
    }

    public void checkTeamPropLocation(TeamPropLocation _location) {
        double dis = getTeamPropDistance();
        for (int i = 0; i < 4; i++) {
            dis += getTeamPropDistance();
        }
        dis = dis / 5.0;
        if (GlobalData.MATCH.AutoFieldLocation == AutoFieldLocation_enum.BACKDROP) { // Backdrop
            if (GlobalData.MATCH.AutoTeamColor == TeamColor.RED) { //Red
                if (_location == TeamPropLocation.CENTER) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;

                    }
                } else if (_location == TeamPropLocation.RIGHT) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;

                    }
                } else if (_location == TeamPropLocation.LEFT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;

                    }
                }
            } else {                                                //Blue
                if (_location == TeamPropLocation.CENTER) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;

                    }
                } else if (_location == TeamPropLocation.LEFT) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;

                    }
                } else if (_location == TeamPropLocation.RIGHT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;

                    }
                }
            }
        } else {                                                                        // WING
            if (GlobalData.MATCH.AutoTeamColor == TeamColor.RED) { //Red
                if (_location == TeamPropLocation.CENTER) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;

                    }
                } else if (_location == TeamPropLocation.LEFT) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;

                    }
                } else if (_location == TeamPropLocation.RIGHT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;

                    }
                }
            } else { //Blue
                if (_location == TeamPropLocation.CENTER) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;

                    }
                } else if (_location == TeamPropLocation.RIGHT) {
                    if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;

                    }
                } else if (_location == TeamPropLocation.LEFT) {
                    if (GlobalData.MATCH.TeamPropLocation == TeamPropLocation.NONE) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;

                    }
                }

            }
        }
    }

    public void checkTeamPropLocation2(TeamPropLocation _location) {
        double dis = getTeamPropDistance();
        for (int i = 0; i < 4; i++) {
            dis += getTeamPropDistance();
        }
        dis = dis / 5.0;
        if (_location == TeamPropLocation.CENTER) {  // Check CENTER first
            if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                GlobalData.MATCH.TeamPropLocation = TeamPropLocation.CENTER;
            }
        } else { // Do the first check
            if (dis < k.ARM.TeamPropMinimumDistance_mm) {
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    if (GlobalData.MATCH.AutoFieldLocation == AutoFieldLocation_enum.BACKDROP) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                    } else {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                    }
                } else if (GlobalData.MATCH.AutoTeamColor == TeamColor.RED) {
                    if (GlobalData.MATCH.AutoFieldLocation == AutoFieldLocation_enum.BACKDROP) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                    }else {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                    }
                }
            }else {
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    if (GlobalData.MATCH.AutoFieldLocation == AutoFieldLocation_enum.BACKDROP) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                    } else {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                    }
                } else if (GlobalData.MATCH.AutoTeamColor == TeamColor.RED) {
                    if (GlobalData.MATCH.AutoFieldLocation == AutoFieldLocation_enum.BACKDROP) {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.LEFT;
                    }else {
                        GlobalData.MATCH.TeamPropLocation = TeamPropLocation.RIGHT;
                    }
                }
            }
        }
    }

    @Override
    public void periodic() {
        armGotoPosition();
        m_opMode.telemetry.addData("Team Prop Distance", "%3.3f", getTeamPropDistance());
        m_opMode.telemetry.addData("Team Prop Location", GlobalData.MATCH.TeamPropLocation);

        m_opMode.telemetry.addData("Shoulder Angle", "%3.3f", m_shoulder.getAngle());
        m_opMode.telemetry.addData("Shoulder Power", "%3.3f", m_shoulder.getPower());
        m_opMode.telemetry.addData("Shoulder Requested Angle", "%3.3f", m_requestedArmAng);

        m_opMode.telemetry.addData("Claw Angle", "%3.3f", m_claw.getClawRotateAngle());
        m_opMode.telemetry.addData("Claw Requested Angle", "%3.3f", m_requestedClawAngle);
//
//        m_opMode.telemetry.addData("Forearm Position", "%3.3f", m_forearm.getPosition());
//        m_opMode.telemetry.addData("Forearm Requested Position ", "%3.3f", m_requestedForearmPosition);
    }
}
