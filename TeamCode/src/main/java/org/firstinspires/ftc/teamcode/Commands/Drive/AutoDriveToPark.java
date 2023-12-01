package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Direction;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

/** Drive to the left or right for a certain time based on where the pixel on the backdrop was placed
 */
public class AutoDriveToPark extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    double m_driveAngle = 0;
    double m_robotAngle = 0;
    Direction m_direction;
    double m_timeOut_sec = 1000;
    double m_speed = 0.4;

    PIDController rotPID;
    Timing.Timer m_elapsedTimer;

    public AutoDriveToPark(CommandOpMode _opMode, AutoDriveSubsystem _drive, Direction _direction) {
        m_opMode = _opMode;
        m_drive = _drive;
        m_direction = _direction;
    }

    @Override
    public void initialize(){
        rotPID = new PIDController(k.DRIVE.Rot_P,k.DRIVE.Rot_I,0);
        rotPID.reset();
        if(GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE){
            m_robotAngle = 90;
            if(m_direction == Direction.RIGHT){
                m_driveAngle = 0;

            }else {
                m_driveAngle = -180;
            }
        }else { // RED
            m_robotAngle = -90;
            if(m_direction == Direction.RIGHT){
                m_driveAngle = -180;
            }else {
                m_driveAngle = 0;
            }
        }
        switch (GlobalData.MATCH.TeamPropLocation) {
            case CENTER:
            case NONE:
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 1.2 : 1.201;
                } else {  // RED
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 1.201 : 1.2;
                }
                break;
            case LEFT:
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 1.5 : 1.5;
                } else {  // RED
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 1.5 : 1.0;
                }
                break;

            case RIGHT:
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 1.0 : 1.7;
                } else {  // RED
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 1.0 : 1.5;
                }
                break;
        }

        m_elapsedTimer =  new Timing.Timer((long)(m_timeOut_sec*1000), TimeUnit.MILLISECONDS);
        m_elapsedTimer.start();
    }
    @Override
    public void execute(){

        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        m_drive.drivePolar(m_driveAngle, m_speed, rot);

    }
    @Override
    public boolean isFinished(){
        if(m_elapsedTimer.done()){
            m_drive.disableMotors();
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean _interrupted){
        m_drive.disableMotors();
    }
}
