package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

/** Drive the robot at a speed, angle for a certain time. Also rotate the robot to an angle while driving
   Since the robot drives at a velocity the only changing variable to get a consistent distance is
   the battery which changes the time it takes to go from stop to full velocity. We hope this change
   will not affect the accuracy to much. There is no PID on distance and the coast time will need
   to be considered.
 */
public class AutoDriveToBackdrop extends CommandBase {
    CommandOpMode m_opMode;
    DriveSubsystem m_drive;
    double m_driveAngle = 0;
    double m_robotAngle = 0;
    int m_timeOut = 1000;
    double m_speed = 0.4;

    PIDController rotPID;
    Timing.Timer m_elapsedTimer;

    public AutoDriveToBackdrop(CommandOpMode _opMode, DriveSubsystem _drive) {
        m_opMode = _opMode;
        m_drive = _drive;
    }

    @Override
    public void initialize(){
        rotPID = new PIDController(k.DRIVE.Rot_P,k.DRIVE.Rot_I,0);
        rotPID.reset();

        switch (GlobalData.TeamPropLocation) {
            case CENTER:
            case NONE:
                if (GlobalData.TeamColor == TeamColor.BLUE) {
                    m_driveAngle = -74;
                    m_robotAngle = 90;
                    m_timeOut = 2200;
                } else {  // RED
                    m_driveAngle = 77;
                    m_robotAngle = -90;
                    m_timeOut = 2700;
                }
                break;
            case LEFT:
                if (GlobalData.TeamColor == TeamColor.BLUE) {
                    m_driveAngle = -73;
                    m_robotAngle = 90;
                    m_timeOut = 2600;
                } else {  // RED
                    m_driveAngle = 73;
                    m_robotAngle = -90;
                    m_timeOut = 2400;
                }
                break;

            case RIGHT:
                if (GlobalData.TeamColor == TeamColor.BLUE) {
                    m_driveAngle = -82;
                    m_robotAngle = 90;
                    m_timeOut = 2600;
                } else {  // RED
                    m_driveAngle = 82;
                    m_robotAngle = -90;
                    m_timeOut = 2500;
                }
                break;
        }

        m_elapsedTimer =  new Timing.Timer(m_timeOut, TimeUnit.MILLISECONDS);
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
