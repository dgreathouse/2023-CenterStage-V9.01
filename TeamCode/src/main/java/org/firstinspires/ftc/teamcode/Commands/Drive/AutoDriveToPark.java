package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Direction;
import com.arcrobotics.ftclib.util.MathUtils;
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
    double m_rotSpeed = 0.3;
    PIDController rotPID;
    Timing.Timer m_timer;

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
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 1.2 : 1.35;
                } else {  // RED
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 2.05 : 2.05;
                }
                break;
            case LEFT:
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 2.5 : 2.5;
                } else {  // RED
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 2.5 : 2.5;
                }
                break;

            case RIGHT:
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 1.7 : 1.7;
                } else {  // RED
                    m_timeOut_sec = (m_direction == Direction.RIGHT) ? 2.0 :2.0;
                }
                break;
        }

        m_timer =  new Timing.Timer((long)(m_timeOut_sec*1000), TimeUnit.MILLISECONDS);
        m_timer.start();
    }
    @Override
    public void execute(){

        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        rot = MathUtils.clamp(rot, -m_rotSpeed, m_rotSpeed);
        double currentSpeed = m_drive.getRampSpeed(m_speed,m_timeOut_sec, m_timer.elapsedTime() / 1000.0, 0.75, 0.75);
        m_drive.drivePolar(m_driveAngle, currentSpeed, rot);

    }
    @Override
    public boolean isFinished(){
        if(m_timer.done()){
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
