package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * Drive to Team Prop Location. The robot starts in the center.
 * The Global variable k.ARM.TeamPropLocation is set by a locator before this command.
 * The location will determine the angle to drive and rotate.
 * Those angles are stored here instead of the k.ARM location.
 */
public class AutoDriveToTeamProp extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    double m_driveAngle = 0;
    double m_robotAngle = 0;
    int m_timeOut = 1000;
    double m_speed = 0.4;

    PIDController rotPID;
    Timing.Timer m_elapsedTimer;

    public AutoDriveToTeamProp(CommandOpMode _opMode, AutoDriveSubsystem _drive) {
        m_opMode = _opMode;
        m_drive = _drive;
    }

    @Override
    public void initialize() {
        rotPID = new PIDController(k.DRIVE.Rot_P, k.DRIVE.Rot_I, 0);
        rotPID.reset();

        switch (GlobalData.TeamPropLocation) {
            case CENTER:
            case NONE:
                if (GlobalData.TeamColor == TeamColor.BLUE) {
                    m_driveAngle = 0;
                    m_robotAngle = -17.5;
                    m_timeOut = 450;
                    m_speed = 0.0;
                } else {
                    m_driveAngle = 0;
                    m_robotAngle = 17.5;
                    m_timeOut = 1000;
                    m_speed = 0.0;
                }
                break;
            case LEFT:
                if (GlobalData.TeamColor == TeamColor.BLUE) {
                    m_driveAngle = -90;
                    m_robotAngle = 40;
                    m_timeOut = 2000;
                    m_speed = 0.0;
                } else {
                    m_driveAngle = -90;
                    m_robotAngle = 45;
                    m_timeOut = 1000;
                    m_speed = 0.0;
                }
                break;

            case RIGHT:
                if (GlobalData.TeamColor == TeamColor.BLUE) {
                    m_driveAngle = 90;
                    m_robotAngle = -45;
                    m_timeOut = 1500;
                    m_speed = 0.0;

                } else {

                    m_driveAngle = 0;
                    m_robotAngle = -45;
                    m_timeOut = 1000;
                    m_speed = 0.0;
                }
                break;
//            case CENTER:
//            case NONE:
//                if(GlobalData.TeamColor == TeamColor.BLUE){
//                    m_driveAngle = 0;
//                    m_robotAngle = 0;
//                    m_timeOut = 700;
//                }else {  // RED
//                    m_driveAngle = 0;
//                    m_robotAngle = 0;
//                    m_timeOut = 700;
//                }
//                break;
//            case LEFT:
//                if(GlobalData.TeamColor == TeamColor.BLUE){
//                    m_driveAngle = -90;
//                    m_robotAngle = 0;
//                    m_timeOut = 500;
//                }else {  // RED
//                    m_driveAngle = -45;
//                    m_robotAngle = 45;
//                    m_timeOut = 500;
//                }
//                break;
//
//            case RIGHT:
//                if(GlobalData.TeamColor == TeamColor.BLUE){
//                    m_driveAngle = 45;
//                    m_robotAngle = -45;
//                    m_timeOut = 1000;
//                }else {  // RED
//                    m_driveAngle = 90;
//                    m_robotAngle = 0;
//                    m_timeOut = 500;
//                }
//                break;

        }

        m_elapsedTimer = new Timing.Timer(m_timeOut, TimeUnit.MILLISECONDS);
        m_elapsedTimer.start();
    }

    @Override
    public void execute() {

        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        m_drive.drivePolar(m_driveAngle, m_speed, rot);

    }

    @Override
    public boolean isFinished() {
        if (m_elapsedTimer.done()) {
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
