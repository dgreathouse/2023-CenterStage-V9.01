package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

/** Drive the robot at a speed, angle for a certain time. Also rotate the robot to an angle while driving
   Since the robot drives at a velocity the only changing variable to get a consistent distance is
   the battery which changes the time it takes to go from stop to full velocity. We hope this change
   will not affect the accuracy to much. There is no PID on distance and the coast time will need
   to be considered.
 */
public class AutoDriveToBackdrop extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    double m_driveAngle = 0;
    double m_robotAngle = 0;
    double m_timeOut_sec = 1.0;
    double m_speed = 0.4;

    PIDController rotPID;
    Timing.Timer m_elapsedTimer;

    public AutoDriveToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive) {
        m_opMode = _opMode;
        m_drive = _drive;
    }

    @Override
    public void initialize(){
        rotPID = new PIDController(k.DRIVE.Rot_P,k.DRIVE.Rot_I,0);
        rotPID.reset();

        switch (GlobalData.MATCH.TeamPropLocation) {
            case CENTER:
            case NONE:
                m_timeOut_sec = 2.6;
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_driveAngle = -70;
                    m_robotAngle = 90;
                } else {  // RED
                    m_driveAngle = 70;
                    m_robotAngle = -90;
                }
                break;
            case LEFT:
                m_timeOut_sec = 2.6;
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_driveAngle = -60;
                    m_robotAngle = 90;
                } else {  // RED
                    m_driveAngle = 65;
                    m_robotAngle = -90;
                }
                break;

            case RIGHT:
                m_timeOut_sec = 2.5;
                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_driveAngle = -79;
                    m_robotAngle = 90;
                } else {  // RED
                    m_driveAngle = 79;
                    m_robotAngle = -90;
                }
                break;
        }

        m_elapsedTimer =  new Timing.Timer((long)(m_timeOut_sec*1000.0), TimeUnit.MILLISECONDS);
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
