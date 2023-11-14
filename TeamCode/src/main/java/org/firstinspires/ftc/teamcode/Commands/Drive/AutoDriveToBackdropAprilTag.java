package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

/** Drive to backdrop using april tags
 * Robot must be stopped in front of the backdrop to read the Apriltags
 * GlobalData.tagOfInterest holds the 1-6 number of the april tag of interest from when we detected.
 * GlobalData.tagPosX,Z contain the location.
 * We need to calculate a drive angle and timeout to get the robot to go to the position.
 * Z = the hyp of the angle, X = the distance left or right from the tag
 *         X
 *        -------
 *        |
 *        |
 *        |  Z
 *        |
 *        |
 *      Robot
 *  Z is used in the timeOut
 */
public class AutoDriveToBackdropAprilTag extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    double m_driveAngle = 0;
    double m_robotAngle = 0;
    int m_timeOut = 1000;
    double m_speed = 0.4;
    double m_ZTimeScale = 2000; // Scale factor for distance of Z travel as timeOut.
    PIDController rotPID;
    Timing.Timer m_elapsedTimer;

    public AutoDriveToBackdropAprilTag(CommandOpMode _opMode, AutoDriveSubsystem _drive) {
        m_opMode = _opMode;
        m_drive = _drive;
    }

    @Override
    public void initialize(){
//        // Calculate the angle from the
        m_driveAngle = -Math.toDegrees(Math.atan(GlobalData.AprilTag_X / GlobalData.AprilTag_Z));
//        // Convert the angle to match the direction of the x for +/-
//        // Not needed the asin function handles a negative number
        double hyp = Math.sqrt(GlobalData.AprilTag_Z * GlobalData.AprilTag_Z + GlobalData.AprilTag_X * GlobalData.AprilTag_X);
//        m_timeOut = (int)(hyp * m_ZTimeScale);
//        GlobalData.aprilTagTime = m_timeOut;
//        rotPID = new PIDController(k.DRIVE.Rot_P,k.DRIVE.Rot_I,0);
//        rotPID.reset();
//
//        if (GlobalData.TeamColor == TeamColor.BLUE) {
//            m_driveAngle -= 90;
//            m_robotAngle = 90;
//        }else {  // Red
//            m_driveAngle = m_driveAngle + 90;
//            GlobalData.aprilTagAngle = m_driveAngle;
//            m_robotAngle = -90;
//        }

        m_elapsedTimer =  new Timing.Timer(m_timeOut, TimeUnit.MILLISECONDS);
        m_elapsedTimer.start();
    }
    @Override
    public void execute(){
        // Calculate the angle from the
        m_driveAngle = -Math.toDegrees(Math.atan(GlobalData.AprilTag_X / GlobalData.AprilTag_Z));
        // Convert the angle to match the direction of the x for +/-
        // Not needed the asin function handles a negative number
        double hyp = Math.sqrt(GlobalData.AprilTag_Z * GlobalData.AprilTag_Z + GlobalData.AprilTag_X * GlobalData.AprilTag_X);
        m_timeOut = (int)(hyp * m_ZTimeScale);
        GlobalData.AprilTagRange = m_timeOut;
        rotPID = new PIDController(k.DRIVE.Rot_P,k.DRIVE.Rot_I,0);
        rotPID.reset();

        if (GlobalData.TeamColor == TeamColor.BLUE) {
            m_driveAngle -= 90;
            m_robotAngle = 90;
        }else {  // Red
            m_driveAngle = m_driveAngle + 90;
            GlobalData.AprilTagBearing = m_driveAngle;
            m_robotAngle = -90;
        }
       // double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        //m_drive.drivePolar(m_driveAngle, m_speed, rot);

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
