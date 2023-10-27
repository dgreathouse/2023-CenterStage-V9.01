package org.firstinspires.ftc.teamcode.Commands.Drone;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DroneSubsystem;

public class DroneDefaultCommand  extends CommandBase {
    CommandOpMode m_opMode;
    DroneSubsystem m_drone;

    public DroneDefaultCommand(CommandOpMode _opMode, DroneSubsystem _drone){
        m_opMode = _opMode;
        m_drone = _drone;
        addRequirements(m_drone);
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        m_drone.spin(0);
    }
    @Override
    public void end(boolean _interrupted){

    }
}
