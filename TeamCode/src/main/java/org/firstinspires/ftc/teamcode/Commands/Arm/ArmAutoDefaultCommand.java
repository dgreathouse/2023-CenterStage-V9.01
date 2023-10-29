package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/** Arm Auto Default Command

 */
public class ArmAutoDefaultCommand extends CommandBase {
    ArmSubsystem m_arm;
    CommandOpMode m_opMode;


    public ArmAutoDefaultCommand(CommandOpMode _opMode, ArmSubsystem _arm){
        m_opMode = _opMode;
        m_arm = _arm;

        addRequirements(m_arm);
    }
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        if(GlobalData.ArmAutoEnable){
            m_arm.armGotoPosition();

        }

    }
    @Override
    public void end(boolean _interrupted){

    }
}
