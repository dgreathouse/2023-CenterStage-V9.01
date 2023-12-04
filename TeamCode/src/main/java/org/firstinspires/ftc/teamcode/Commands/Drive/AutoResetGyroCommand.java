package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoResetGyroCommand extends CommandBase {
    CommandOpMode m_opMode;
    AutoDriveSubsystem m_drive;
    boolean m_isFinished = false;
    double m_delay_sec;
    Timing.Timer m_timer;
    public AutoResetGyroCommand(CommandOpMode _opMode, AutoDriveSubsystem _drive){
        m_opMode = _opMode;
        m_drive = _drive;
        m_delay_sec = 0.05; // Same delay that is in the Reset Yaw method.

    }
    @Override
    public void initialize(){
        m_timer = new Timing.Timer((long)(m_delay_sec*1000.0), TimeUnit.MILLISECONDS);
        m_timer.start();
        m_drive.resetYaw();
    }
    @Override
    public void execute(){
        if(m_timer.done()){
            m_isFinished = true;
        }
    }
    @Override
    public boolean isFinished(){

        return m_isFinished;
    }
}
