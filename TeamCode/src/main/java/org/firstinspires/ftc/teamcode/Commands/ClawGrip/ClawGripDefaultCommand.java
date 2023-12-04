package org.firstinspires.ftc.teamcode.Commands.ClawGrip;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawGripSubsystem;

/** Arm Default Command
 * This command is used during Driver Control
 * Responsible for:
 * o - Reading joystick buttons
 * o - Commanding the Shoulder, Forearm, Claw
 *
 */
public class ClawGripDefaultCommand extends CommandBase {
    ClawGripSubsystem m_claw;
    CommandOpMode m_opMode;
    TriggerReader clawClose;
    TriggerReader lowerPixelRelease;

    public ClawGripDefaultCommand(CommandOpMode _opMode, ClawGripSubsystem _claw){
        m_opMode = _opMode;
        m_claw = _claw;

        addRequirements(m_claw);
    }
    @Override
    public void initialize(){
        clawClose = new TriggerReader(Hw.s_gpOperator,GamepadKeys.Trigger.LEFT_TRIGGER);
        lowerPixelRelease = new TriggerReader(Hw.s_gpOperator,GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    @Override
    public void execute(){

        m_claw.setClawGripAngle(m_claw.getClawGripAngle());
        // Manage the Claw
        clawClose.readValue();
        if(clawClose.isDown()){
            m_claw.setClawGripAngle(m_claw.getClawCloseAngle());
        }
        lowerPixelRelease.readValue();
        if(lowerPixelRelease.isDown()){
            m_claw.setClawGripAngle(m_claw.getClawReleaseLowerAngle());
        }

    }
    @Override
    public void end(boolean _interrupted){

    }
}
