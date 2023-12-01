package org.firstinspires.ftc.teamcode.Commands.Auto;

import android.widget.Switch;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoDropYellowCenter extends CommandBase {
    CommandOpMode m_opMode;
    boolean m_isFinished = false;
    double m_delay_sec;
    Timing.Timer m_timer;
    public AutoDropYellowCenter(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw, double _time_sec){
        m_opMode = _opMode;
        m_delay_sec = _time_sec;
    }
    @Override
    public void initialize(){
        m_timer = new Timing.Timer((long)(m_delay_sec*1000.0), TimeUnit.MILLISECONDS);
        m_timer.start();
    }
    @Override
    public void execute(){
        double driveAngle = 5;
        driveAngle = GlobalData.MATCH.AutoTeamColor == TeamColor.RED ? driveAngle: driveAngle * -1.0;

        AutoDriveTimeVel a;


        if(m_timer.done() || m_isFinished == true){
            m_isFinished = true;
        }
    }
    @Override
    public boolean isFinished(){

        return m_isFinished;
    }
}
