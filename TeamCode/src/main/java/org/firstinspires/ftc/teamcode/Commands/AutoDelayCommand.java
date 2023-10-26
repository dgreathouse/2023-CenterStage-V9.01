package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class AutoDelayCommand extends CommandBase {
    CommandOpMode m_opMode;
    boolean m_isFinished = false;
    int m_delay_ms = 0;
    Timing.Timer m_timer;
    public AutoDelayCommand(CommandOpMode _opMode, int _time_ms){
        m_opMode = _opMode;
        m_delay_ms = _time_ms;
    }
    @Override
    public void initialize(){
        m_timer = new Timing.Timer(m_delay_ms, TimeUnit.MILLISECONDS);
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
