package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.DAngle;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

/** Drive the robot at a speed, angle for a certain time. Also rotate the robot to an angle while driving
   Since the robot drives at a velocity the only changing variable to get a consistent distance is
   the battery which changes the time it takes to go from stop to full velocity. We hope this change
   will not affect the accuracy to much. There is no PID on distance and the coast time will need
   to be considered.
 */
public class AutoDriveTimeVel extends CommandBase {
    CommandOpMode m_opMode;
    DriveSubsystem m_drive;
    double m_angle;
    double m_robotAngle;
    int m_timeOut;
    double m_speed;

    PIDController rotPID = new PIDController(.01,0,0);
    Timing.Timer m_timer;


    public AutoDriveTimeVel(CommandOpMode _opMode, DriveSubsystem _drive, double _angle, double _speed, double _robotAngle, int _timeOut) {
        m_opMode = _opMode;
        m_drive = _drive;
        m_angle = _angle;
        m_speed = _speed;
        m_robotAngle = _robotAngle;
        m_timeOut = _timeOut;
    }

    @Override
    public void initialize(){
        rotPID.reset();
        m_drive.resetMotors();
        m_timer = new Timing.Timer(m_timeOut, TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void execute(){
        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        m_drive.drivePolar(m_angle, m_speed, rot);
    }

    @Override
    public boolean isFinished(){
        if(m_timer.done()){
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean _interrupted){
        m_drive.disableMotors();
    }
}
