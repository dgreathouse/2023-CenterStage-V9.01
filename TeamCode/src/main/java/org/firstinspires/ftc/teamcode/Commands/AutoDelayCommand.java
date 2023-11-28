package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class AutoDelayCommand extends CommandBase {
    CommandOpMode m_opMode;
    boolean m_isFinished = false;
    double m_delay_sec;
    Timing.Timer m_timer;
    public AutoDelayCommand(CommandOpMode _opMode, double _time_sec){
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
        if(m_timer.done()){
            m_isFinished = true;
        }
    }
    @Override
    public boolean isFinished(){

        return m_isFinished;
    }
}
