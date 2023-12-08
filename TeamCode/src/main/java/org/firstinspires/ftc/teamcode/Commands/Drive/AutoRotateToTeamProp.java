package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * Drive to Team Prop Location. The robot starts in the center.
 * The Global variable k.ARM.TeamPropLocation is set by a locator before this command.
 * The location will determine the angle to drive and rotate.
 * Those angles are stored here instead of the k.ARM location.
 */
public class AutoRotateToTeamProp extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;

    double m_robotAngle = 0;
    int m_timeOut = 3000;
    double m_speed = 0.25;

    PIDController rotPID;
    Timing.Timer m_timer;

    public AutoRotateToTeamProp(CommandOpMode _opMode, AutoDriveSubsystem _drive) {
        m_opMode = _opMode;
        m_drive = _drive;
    }

    @Override
    public void initialize() {
        rotPID = new PIDController(.008, .003, 0);
        rotPID.setTolerance(2.0);
        rotPID.reset();

        switch (GlobalData.MATCH.TeamPropLocation) {
            case CENTER:
            case NONE:
                m_robotAngle = GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE ? 25 : -25;
                break;
            case LEFT:
                m_robotAngle = 85;
                break;
            case RIGHT:
                m_robotAngle = -85;
                break;

        }

        m_timer = new Timing.Timer(m_timeOut, TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void execute() {

        double rot = -rotPID.calculate(m_drive.getRobotAngle(), m_robotAngle);
        rot = MathUtils.clamp(rot,-m_speed,m_speed);
        m_drive.drivePolar(0, 0, rot);

    }

    @Override
    public boolean isFinished() {
        if(m_timer.done() || rotPID.atSetPoint()){
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
