package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
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
    double m_speed = 0.7;
    double m_rotSpeed = 0.5;
    double m_rampUpTime_sec = 0.750;
    double m_rampDownTime_sec = 0.750;
    PIDController rotPID;
    Timing.Timer m_timer;

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

                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_driveAngle = -62;
                    m_robotAngle = 90;
                    m_timeOut_sec = 2.025;
                } else {  // RED
                    m_driveAngle = 63;
                    m_robotAngle = -90;
                    m_timeOut_sec = 2.025;
                }
                break;
            case LEFT:

                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_driveAngle = -55;   // Large angle short distance
                    m_robotAngle = 90;
                    m_timeOut_sec = 2.1;
                } else {  // RED
                    m_driveAngle = 55;    // Small angle long distance
                    m_robotAngle = -90;
                    m_timeOut_sec = 2.1;
                }
                break;

            case RIGHT:

                if (GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE) {
                    m_driveAngle = -71;  // Small angle long distance
                    m_robotAngle = 90;
                    m_timeOut_sec = 2;
                } else {  // RED
                    m_driveAngle = 71;   // Large angle short distance
                    m_robotAngle = -90;
                    m_timeOut_sec = 2;
                }
                break;
        }
        if (m_timeOut_sec >= 2.0) {
            m_rampUpTime_sec = 0.75;
            m_rampDownTime_sec = 0.75;
        } else {
            m_rampUpTime_sec = m_timeOut_sec * 0.50;
            m_rampUpTime_sec = MathUtils.clamp(m_rampUpTime_sec, 0, 0.75);
            m_rampDownTime_sec = m_timeOut_sec * 0.50;
            m_rampDownTime_sec = MathUtils.clamp(m_rampDownTime_sec, 0, 0.75);
        }
        m_timer =  new Timing.Timer((long)(m_timeOut_sec*1000.0), TimeUnit.MILLISECONDS);
        m_timer.start();
    }
    @Override
    public void execute(){

        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        rot = MathUtils.clamp(rot, -m_rotSpeed, m_rotSpeed);
        double currentSpeed = m_drive.getRampSpeed(m_speed,m_timeOut_sec, m_timer.elapsedTime() / 1000.0, m_rampUpTime_sec, m_rampDownTime_sec);
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
