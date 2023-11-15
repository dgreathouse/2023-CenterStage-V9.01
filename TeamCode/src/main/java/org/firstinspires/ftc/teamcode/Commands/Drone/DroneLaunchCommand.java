package org.firstinspires.ftc.teamcode.Commands.Drone;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.DroneSubsystem;

import java.util.concurrent.TimeUnit;

public class DroneLaunchCommand extends CommandBase {
    CommandOpMode m_opMode;
    DroneSubsystem m_drone;
    Timing.Timer m_timer;
    double m_speed = 1.0;
    public DroneLaunchCommand(CommandOpMode _opMode, DroneSubsystem _drone){
        m_opMode = _opMode;
        m_drone = _drone;
        addRequirements(m_drone);
    }
    @Override
    public void initialize(){
        m_timer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
        m_timer.start();
    }
    @Override
    public void execute(){
        m_drone.spin(m_speed);
    }
    @Override
    public void end(boolean _interrupted){

    }
    @Override
    public boolean isFinished(){
        if(m_timer.done()){
            m_drone.spin(0);
            return true;
        }
        return false;
    }
}
