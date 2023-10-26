package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

/** Drive the robot at a speed, angle for a certain time. Also rotate the robot to an angle while driving
   Since the robot drives at a velocity the only changing variable to get a consistent distance is
   the battery which changes the time it takes to go from stop to full velocity. We hope this change
   will not affect the accuracy to much. There is no PID on distance and the coast time will need
   to be considered.
 */
public class AutoRotateRobotToTeamProp extends CommandBase {
    CommandOpMode m_opMode;
    DriveSubsystem m_drive;
    double m_angle;
    int m_timeOut;
    double m_speed;
    boolean isFinished = false;
    PIDController rotPID = new PIDController(0,0,0);
    Timing.Timer m_timer;


    public AutoRotateRobotToTeamProp(CommandOpMode _opMode, DriveSubsystem _drive, double _angle, double _speed, int _timeOut) {
        m_opMode = _opMode;
        m_drive = _drive;
        m_angle = _angle;
        m_speed = _speed;
        m_timeOut = _timeOut;
    }

    @Override
    public void initialize(){
        rotPID.setPID(0.0075,0.05,0);
        rotPID.setTolerance(1.0);
        //rotPID.setIntegrationBounds(-0.1,0.1);

        rotPID.reset();
        m_drive.resetMotors();
        m_timer = new Timing.Timer(m_timeOut, TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void execute(){
        if(GlobalData.TeamPropDistanceCenter > 1 && GlobalData.TeamPropDistanceCenter < 500){
            isFinished = true;
        }else {
            double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_angle);
            rot = MathUtils.clamp(rot,-m_speed,m_speed);
            m_drive.driveXY(0, 0, rot);
        }

    }

    @Override
    public boolean isFinished(){
        if(m_timer.done() || rotPID.atSetPoint()){
            isFinished = true;
        }
        return isFinished;
    }
    @Override
    public void end(boolean _interrupted){
        m_drive.disableMotors();
    }
}
