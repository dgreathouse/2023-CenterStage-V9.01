package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

import java.util.concurrent.TimeUnit;

/**
 */
public class ArmGetTeamPropLocation extends CommandBase {
    CommandOpMode m_opMode;
    ArmSubsystem m_arm;
    Timing.Timer m_servoTiming;
    boolean isFinished = false;
    public ArmGetTeamPropLocation(CommandOpMode _opMode, ArmSubsystem _arm) {
        m_opMode = _opMode;
        m_arm = _arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){
       m_servoTiming = new Timing.Timer(1000, TimeUnit.MILLISECONDS);

    }
    @Override
    public void execute(){

        double dis = m_arm.getTeamPropDistance();
        m_opMode.telemetry.addData("dis",dis);
        if(dis > k.ARM.TeamPropMinimumDistance_mm){
            m_arm.setTeamPropServo(.1);
            m_servoTiming.start();
            while(!m_servoTiming.done()){
                dis = m_arm.getTeamPropDistance();
            }
            if(dis > k.ARM.TeamPropMinimumDistance_mm){
                k.ARM.TeamPropLoc = TeamPropLocation.RIGHT;
            }else {
                k.ARM.TeamPropLoc  = TeamPropLocation.LEFT;
            }
        }
        isFinished = true;
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }
    @Override
    public void end(boolean _interrupted){

    }
}
