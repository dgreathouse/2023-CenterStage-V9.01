package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoDriveToDistance extends CommandBase
{
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    boolean m_isFinished = false;
    Timing.Timer m_timer;
    double m_timeOut_sec;
    double m_robotAngle;
    double m_driveAngle;
    double m_distance;
    double m_speed;
    double m_rampUpTime_sec = 1.0; //Seconds
    double m_rampDownTime_sec = 0;
    double m_currentSpeed;
    PIDController rotPID = new PIDController(0.01,0,0);
    PIDController drvPID = new PIDController(0.01,0.01,0);
    public AutoDriveToDistance(CommandOpMode _opMode, AutoDriveSubsystem _drive, double _distance_mm, double _speed, double _driveAngle, double _robotAngle, double _timeOut_sec){
        m_opMode = _opMode;
        m_drive = _drive;
        m_timeOut_sec = _timeOut_sec;
        m_robotAngle = _robotAngle;
        m_driveAngle = _driveAngle;
        m_distance = _distance_mm;
        m_speed = _speed;
        if (m_timeOut_sec >= 2.0) {
            m_rampUpTime_sec = 0.75;
        } else {
            m_rampUpTime_sec = m_timeOut_sec * 0.50;
            m_rampUpTime_sec = MathUtils.clamp(m_rampUpTime_sec, 0, 0.75);
        }
    }

    @Override
    public void initialize(){
        rotPID.reset();
        rotPID.setTolerance(0.5);
        drvPID.reset();
        drvPID.setTolerance(1);
        drvPID.setIntegrationBounds(-0.2,0.2);
        m_timer = new Timing.Timer((long)(m_timeOut_sec * 1000000), TimeUnit.MICROSECONDS);
        m_timer.start();
    }
    @Override
    public void execute(){
        double rot = -rotPID.calculate(m_drive.getRobotAngle(),m_robotAngle);
        double drv = drvPID.calculate(m_drive.getCurrentPosition(), m_distance);
        drv = MathUtils.clamp(drv, -m_speed, m_speed);

        double currentTime_sec = m_timer.elapsedTime() / 1000000.0;
        m_currentSpeed = m_drive.getRampSpeed(drv, m_timeOut_sec, currentTime_sec,m_rampUpTime_sec,m_rampDownTime_sec);
        m_drive.drivePolar(m_driveAngle,m_currentSpeed, rot);
        if(m_timer.done() || drvPID.atSetPoint()){
            m_isFinished = true;
        }
    }
    @Override
    public boolean isFinished(){
        return m_isFinished;
    }
    @Override
    public void end(boolean _interrupted){
        m_drive.disableMotors();
    }
}
