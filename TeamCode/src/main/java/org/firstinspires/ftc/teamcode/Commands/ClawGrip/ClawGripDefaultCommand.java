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
    TriggerReader lowerPixelRelease;
    TriggerReader upperPixelRelease;

    public ClawGripDefaultCommand(CommandOpMode _opMode, ClawGripSubsystem _claw){
        m_opMode = _opMode;
        m_claw = _claw;

        addRequirements(m_claw);
    }
    @Override
    public void initialize(){
        lowerPixelRelease = new TriggerReader(Hw.s_gpOperator,GamepadKeys.Trigger.LEFT_TRIGGER);
        upperPixelRelease = new TriggerReader(Hw.s_gpOperator,GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    @Override
    public void execute(){

        m_claw.setClawGripAngle(m_claw.getClawGripAngle());
        // Manage the Claw
        lowerPixelRelease.readValue();
        if(lowerPixelRelease.isDown()){
            m_claw.setClawGripAngle(m_claw.getClawReleaseLowerAngle());
        }
        upperPixelRelease.readValue();
        if(upperPixelRelease.isDown()){
            m_claw.setClawGripAngle(m_claw.getClawReleaseUpperAngle());
        }

    }
    @Override
    public void end(boolean _interrupted){

    }
}
