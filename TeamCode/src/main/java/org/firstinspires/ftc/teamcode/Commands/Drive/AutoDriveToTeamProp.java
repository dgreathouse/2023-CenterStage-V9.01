package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

/** Drive to Team Prop Location. The robot starts in the center.
 * The Global variable k.ARM.TeamPropLocation is set by a locator before this command.
 * The location will determine the angle to drive and rotate.
 * Those angles are stored here instead of the k.ARM location.
 *
 */
public class AutoDriveToTeamProp extends CommandBase {
    CommandOpMode m_opMode;
    DriveSubsystem m_drive;
    double m_angle = 0;
    double m_robotAngle = 0;
    int m_timeOut = 1;
    double m_speed = 0.5;

    PIDController rotPID = new PIDController(.01,0,0);
    Timing.Timer m_elapsedTimer;
    //ElapsedTime m_elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public AutoDriveToTeamProp(CommandOpMode _opMode, DriveSubsystem _drive) {
        m_opMode = _opMode;
        m_drive = _drive;

    }

    @Override
    public void initialize(){
        rotPID.reset();
        m_drive.resetMotors();

        m_speed = 0.5;
        switch (k.ARM.TeamPropLoc){
            case CENTER:
                m_angle = 0;
                m_robotAngle = 0;
                m_timeOut = 1;
            break;
            case LEFT:
                m_angle = 1;
                m_robotAngle = 1;
                m_timeOut = 1;
                break;
            case RIGHT:
                m_angle = 2;
                m_robotAngle = 2;
                m_timeOut = 1;
                break;
            default:
                break;

        }
        m_elapsedTimer =  new Timing.Timer(m_timeOut, TimeUnit.MILLISECONDS);
        m_elapsedTimer.start();
    }
    @Override
    public void execute(){

        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        m_drive.drivePolar(m_angle, m_speed, rot);

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
