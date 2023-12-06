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
public class AutoDriveWithDistanceSensor extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    double m_driveAngle;
    double m_robotAngle;
    double m_timeOut_sec;
    double m_speed;
    double m_distance_mm;
    double m_rotSpeed = 0.3;


    PIDController rotPID = new PIDController(0.005, 0.001, 0);
    Timing.Timer m_timer;

    /**
     *
     * @param _opMode
     * @param _drive
     * @param _driveAngle The angle the robot should drive at (-) CCW and (+) CW
     * @param _speed The speed rom 0-1 to drive at
     * @param _robotAngle The angle the robot should face (-) CW and (+) CCW
     * @param _distance_mm The distance to stop at from the distance sensor reading
     * @param _timeOut_sec The safety factor time to end  if the distance is not reached.
     */
    public AutoDriveWithDistanceSensor(CommandOpMode _opMode, AutoDriveSubsystem _drive, double _driveAngle, double _speed, double _robotAngle, double _distance_mm, double _timeOut_sec) {
        m_opMode = _opMode;
        m_drive = _drive;
        m_driveAngle = _driveAngle;
        m_speed = _speed;
        m_robotAngle = _robotAngle;
        m_timeOut_sec = _timeOut_sec;
        m_distance_mm = _distance_mm;
    }

    @Override
    public void initialize() {
        rotPID.reset();
        rotPID.setTolerance(0.05);
        rotPID.setIntegrationBounds(-0.2, 0.2);
        m_timer = new Timing.Timer((long) (m_timeOut_sec * 1000.0), TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void execute() {
        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        rot = MathUtils.clamp(rot, -m_rotSpeed, m_rotSpeed);
        m_drive.drivePolar(m_driveAngle, m_speed, rot);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.done() || m_drive.getDistanceSensorValue() < m_distance_mm) {
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
