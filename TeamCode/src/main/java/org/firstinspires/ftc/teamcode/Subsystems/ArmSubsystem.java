package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;

public class ArmSubsystem extends SubsystemBase {

    private CommandOpMode m_opMode;
    private Claw m_claw;
    private Shoulder m_shoulder;
    private Forearm m_forearm;
    private ServoEx m_teamPropServo;
    public ArmPos m_armPos = ArmPos.NONE;

    public ArmSubsystem(CommandOpMode _opMode) {
        m_opMode = _opMode;
        initHardware();
    }

    /**
     * Initialize the hardware for this module.
     * This is called during the constructor
     */
    private void initHardware() {
        m_claw = new Claw(m_opMode);
        m_shoulder = new Shoulder(m_opMode);
        m_forearm = new Forearm(m_opMode);
    }

    /**
     * Set the arm, shoulder and Claw position with an enum value
     *
     * @param _pos
     */
    public void armGotoPosition(ArmPos _pos) {
        switch (_pos) {
            case STRAIGHT:
                m_forearm.setPosition(0);
                m_shoulder.sePosition(30);
                m_claw.setClawRotateAngle(0,.5);
                break;
            case UP:
                m_forearm.setPosition(0);
                m_shoulder.sePosition(30);
                m_claw.setClawRotateAngle(180,.5);
                break;
            case FLOOR:
                m_forearm.setPosition(0);
                m_shoulder.sePosition(0);
                m_claw.setClawRotateAngle(1,.5);
                break;
            case ANGLE_30:
                m_forearm.setPosition(0);
                m_shoulder.sePosition(30);
                m_claw.setClawRotateAngle(3,.5);
                break;
            case NONE:
                // Set the shoulder to Power mode instead of PID mode.
                // Adjust the claw angle to match the shoulder angle and backdrop angle after off the floor.
                break;
            default:
                break;
        }
    }
    public void armShoulderMove(double _speed){
        double pos = m_shoulder.getPosition();
        // TODO: set scale factor of shoulder position to match the 30 degree angle while shoulder moves
        m_claw.setClawRotateAngle(pos * 1.0, 0.5);
        m_shoulder.move(_speed);
    }
    public void armForearmMove(double _speed){
        m_forearm.move(_speed);
    }
    // Team Prop functions
    void setTeamPropServo(int _pos){
        m_teamPropServo.setPosition(_pos);
    }
    public void getTeamPropLocation(){
        /*TODO:  Read Distance Sensor, If Team Prop located return LEFT, Else Move Servo to
            the Center and if Team Prop located return Center, else return right. Before reading the
            next position you must wait for the servo to move.
        */
        
        // Set the correct value for the location of the team prop from the if statements
        GlobalData.s_teamPropLocation = TeamPropLocation.CENTER;
    }
    // CLAW functions
    public void setClawGripAngle(double _angle){
        m_claw.setClawGripAngle(_angle);
    }

    @Override
    public void periodic() {

    }
}
