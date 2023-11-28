package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

/** Drive the robot at a speed, angle for a certain time. Also rotate the robot to an angle while driving
   Since the robot drives at a velocity the only changing variable to get a consistent distance is
   the battery which changes the time it takes to go from stop to full velocity. We hope this change
   will not affect the accuracy to much. There is no PID on distance and the coast time will need
   to be considered.
 */
public class AutoRotateRobot extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    double m_angle;
    double m_timeOut_sec;
    double m_speed;

    PIDController rotPID = new PIDController(k.DRIVE.Rot_P,k.DRIVE.Rot_I,0);
    Timing.Timer m_timer;


    public AutoRotateRobot(CommandOpMode _opMode, AutoDriveSubsystem _drive, double _angle, double _speed, double _timeOut_sec) {
        m_opMode = _opMode;
        m_drive = _drive;
        m_angle = _angle;
        m_speed = _speed;
        m_timeOut_sec = _timeOut_sec;
    }

    @Override
    public void initialize(){
        rotPID.reset();
        rotPID.setTolerance(1.0, 1.0);
        m_timer = new Timing.Timer((long)(m_timeOut_sec*1000.0), TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void execute(){
        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_angle);
        rot = MathUtils.clamp(rot,-m_speed,m_speed);
        m_drive.driveXY(0, 0, rot);
    }

    @Override
    public boolean isFinished(){
        if(m_timer.done() || rotPID.atSetPoint()){
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
