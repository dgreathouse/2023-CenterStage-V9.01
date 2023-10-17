package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/** Arm Default Command
 * This command is used during Driver Control
 * Responsible for:
 * o - Reading joystick buttons
 * o - Commanding the Shoulder, Forearm, Claw and driving LEDs.
 *
 */
public class ArmDefaultCommand extends CommandBase {
    ArmSubsystem m_arm;
    CommandOpMode m_opMode;
    double m_DB = 0.2;
    TriggerReader lowerPixelRelease;
    TriggerReader upperPixelRelease;
    public ArmDefaultCommand(CommandOpMode _opMode, ArmSubsystem _arm){
        m_opMode = _opMode;
        m_arm = _arm;

        addRequirements(m_arm);
    }
    @Override
    public void initialize(){
        lowerPixelRelease = new TriggerReader(Hw.s_gpOperator,GamepadKeys.Trigger.LEFT_TRIGGER);
        upperPixelRelease = new TriggerReader(Hw.s_gpOperator,GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    @Override
    public void execute(){
        // Arm movement
        // Buttons and Axis move the arm. If the axis is greater than a particular value the button
        // PID values are canceled by putting the ArmPos to NONE.
        // The angle of the Claw will always be at 30 degrees if it is not Straight or Floor
        // This here only handles the Axis movement and the buttons are done in the OpMode

        if(Math.abs(Hw.s_gpOperator.getLeftX()) > m_DB || Math.abs(Hw.s_gpOperator.getLeftY()) > m_DB) {
            m_arm.m_armPos = ArmPos.NONE;
        }
        switch (m_arm.m_armPos){
            case FLOOR:
                m_arm.armGotoPosition(ArmPos.FLOOR);
                break;
            case STRAIGHT:
                m_arm.armGotoPosition(ArmPos.STRAIGHT);
                break;
            case UP:
                m_arm.armGotoPosition(ArmPos.UP);
                break;
            case ANGLE_30:
                m_arm.armGotoPosition(ArmPos.ANGLE_30);
                break;
            case NONE:
                double x = Hw.s_gpOperator.getLeftX();
                double y = Hw.s_gpOperator.getLeftY();
                if(Math.abs(x) > m_DB) {
                    x = Math.signum(x) * (Math.abs(x) - m_DB);
                    m_arm.armForearmMove((x));
                }
                if(Math.abs(y) > m_DB){
                    y = Math.signum(y) * (Math.abs(y) - m_DB);
                    m_arm.armShoulderMove(y);
                }
                break;
            default:
                break;

        }
        // Manage the Claw
        lowerPixelRelease.readValue();
        if(lowerPixelRelease.isDown()){
            m_arm.setClawGripAngle(k.CLAW.OpenLowerAngle);
        }
        upperPixelRelease.readValue();
        if(upperPixelRelease.isDown()){
            m_arm.setClawGripAngle(k.CLAW.OpenUpperAngle);
        }

    }
    @Override
    public void end(boolean _interrupted){

    }
}
