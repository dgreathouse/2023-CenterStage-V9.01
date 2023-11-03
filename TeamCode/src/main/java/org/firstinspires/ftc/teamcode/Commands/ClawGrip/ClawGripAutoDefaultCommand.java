package org.firstinspires.ftc.teamcode.Commands.ClawGrip;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawGripSubsystem;

/** Arm Default Command
 * This command is used during Driver Control
 * Responsible for:
 * o - Reading joystick buttons
 * o - Commanding the Shoulder, Forearm, Claw
 *
 */
public class ClawGripAutoDefaultCommand extends CommandBase {
    ClawGripSubsystem m_claw;
    CommandOpMode m_opMode;


    public ClawGripAutoDefaultCommand(CommandOpMode _opMode, ClawGripSubsystem _claw){
        m_opMode = _opMode;
        m_claw = _claw;

        addRequirements(m_claw);
    }
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        m_claw.setClawGripAngle(GlobalData.ClawAngleAuto);


    }
    @Override
    public void end(boolean _interrupted){

    }
}
