package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/** Arm Default Command
 * This command is used during Driver Control
 * Responsible for:
 * o - Reading joystick buttons
 * o - Commanding the Shoulder, Forearm, Claw
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

        // The Claw will stay parallel to the ground if the shoulder is less than 10 degrees.
        // Over 10 degrees of shoulder movement will place the claw at 30 degrees for the backdrop.
        // This class command can be used by autonomous by just setting the angle needed arm.armSetPosition(_pos
        // Buttons must set angle for Straight, Floor, 5 Stack and 3 Stack.
        // Axis will set the angle. A button to turn on Forearm move Override with Right Stick Movement
        // The forearm will extend as needed when the shoulder is greater than 90 degrees.
        // Start position for encoder is 0. This will be offset to correct angle at startup.


        // Get the x,y value
        double x = Hw.s_gpOperator.getLeftX();
        double y = Hw.s_gpOperator.getLeftY();
        m_opMode.telemetry.addData("xR", x);
        m_opMode.telemetry.addData("yR", y);
        if(Math.hypot(x,y) > 0.8) {                                     // If X and Y is at the edge
            double ang = (125.0/180.0) * (180.0 - Math.toDegrees(Math.atan2(Math.abs(x),y)));     // Find Angle
            m_opMode.telemetry.addData("xyA", ang);
            m_arm.setArmAngle(ang);  // Set the angle in the ArmSubsystem
        }
        // Stop the position movement if START button pushed. Move forearm to climb
        if(!Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get()){
            m_arm.armGotoPosition();
        }else {
            double ry = Hw.s_gpOperator.getRightY();
            m_arm.armForearmMove(ry);
        }


        // Manage the Claw
        lowerPixelRelease.readValue();
        if(lowerPixelRelease.isDown()){
            m_arm.setClawGripAngle(m_arm.getClawReleaseLowerAngle());
        }
        upperPixelRelease.readValue();
        if(upperPixelRelease.isDown()){
            m_arm.setClawGripAngle(m_arm.getClawReleaseUpperAngle());
        }
        // Based on the shoulder angle when above a certain point the claw must stay at 30 degrees.


    }
    @Override
    public void end(boolean _interrupted){

    }
}
