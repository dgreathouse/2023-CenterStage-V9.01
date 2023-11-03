package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Subsystems.ArmAutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/** Arm Auto Default Command

 */
public class ArmAutoDefaultCommand extends CommandBase {
    ArmAutoSubsystem m_arm;
    CommandOpMode m_opMode;
    int x = 0;

    public ArmAutoDefaultCommand(CommandOpMode _opMode, ArmAutoSubsystem _arm){
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
           // m_opMode.telemetry.addData("HHHHHHHHHHHH",x++);
        }

    }
    @Override
    public void end(boolean _interrupted){

    }
}
