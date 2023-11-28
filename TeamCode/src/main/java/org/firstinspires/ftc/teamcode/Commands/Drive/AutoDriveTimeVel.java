package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * Drive the robot at a speed, angle for a certain time. Also rotate the robot to an angle while driving
 * Since the robot drives at a velocity the only changing variable to get a consistent distance is
 * the battery voltage which changes the time it takes to go from stop to full velocity. We hope this change
 * will not affect the accuracy to much. There is no PID on distance and the coast time will need
 * to be considered.
 */
public class AutoDriveTimeVel extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    double m_driveAngle;
    double m_robotAngle;
    double m_timeOut_sec;
    double m_speed;
    double m_currentSpeed;

    double m_rotSpeed = 0.3;
    double m_rampUpTime_sec = 1.0; //Seconds
    double m_rampDownTime_sec = 0.5;

    PIDController rotPID = new PIDController(.025, 0.005, 0);
    Timing.Timer m_timer;


    public AutoDriveTimeVel(CommandOpMode _opMode, AutoDriveSubsystem _drive, double _driveAngle, double _speed, double _robotAngle, double _timeOut_sec, double _rampUpTime_sec, double _rampDownTime_sec) {
        m_opMode = _opMode;
        m_drive = _drive;
        m_driveAngle = _driveAngle;
        m_speed = _speed;
        m_robotAngle = _robotAngle;
        m_timeOut_sec = _timeOut_sec;
        m_currentSpeed = m_speed;

    }

    @Override
    public void initialize() {
        rotPID.reset();
        rotPID.setTolerance(1.0);
        m_timer = new Timing.Timer((long)(m_timeOut_sec*1000.0), TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void execute() {
        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        double currentTime_sec = m_timer.elapsedTime() / 1000.0;

        // Try to gently ramp the speed up from 0 to m_speed

        if (currentTime_sec < m_timeOut_sec && currentTime_sec > m_timeOut_sec - m_rampDownTime_sec) { // In the ramp down time
            m_currentSpeed = m_speed * (m_timeOut_sec - currentTime_sec) / m_rampDownTime_sec;
        } else if (currentTime_sec < m_rampUpTime_sec) {// In the ramp up time
            m_currentSpeed = m_speed * currentTime_sec / m_rampUpTime_sec;
        } else { // past the ramp up time and not in ramp down time
            m_currentSpeed = m_speed;
        }

        rot = MathUtils.clamp(rot, -m_rotSpeed, m_rotSpeed);
        m_drive.drivePolar(m_driveAngle, m_currentSpeed, rot);
        m_opMode.telemetry.addData("CurrentSpeed", m_currentSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.done()) {
            m_drive.disableMotors();
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean _interrupted) {
        m_drive.disableMotors();
    }
}
