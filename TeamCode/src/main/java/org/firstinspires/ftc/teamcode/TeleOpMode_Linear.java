/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drone.DroneDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drone.DroneLaunchCommand;
import org.firstinspires.ftc.teamcode.Commands.LED.LEDDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LEDSubsystem;

import java.util.concurrent.TimeUnit;


/**

 */

@TeleOp(name="TeleOp 1", group="Linear Opmode")

public class TeleOpMode_Linear extends CommandOpMode {
    Timing.Timer m_timer;

    double m_timerAvg = 0.0;
    double m_timerCnt = 0;
    double m_loopRate = 0;
    private Hw hw;
    private DriveSubsystem m_drive;
    private ArmSubsystem m_arm;
    private DroneSubsystem m_drone;
    private LEDSubsystem m_led;
    // Declare OpMode members.

    @Override
    public void initialize() {
        hw = new Hw(this);
        hw.init();

        // Create subsystems
        m_drive = new DriveSubsystem(this);
        m_arm = new ArmSubsystem(this);
        m_drone = new DroneSubsystem(this);
        m_led = new LEDSubsystem(this);

        // Create Default Commands
        DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(this, m_drive);
        ArmDefaultCommand armDefaultCommand = new ArmDefaultCommand(this,m_arm);
        DroneDefaultCommand droneDefaultCommand = new DroneDefaultCommand(this,m_drone);
        LEDDefaultCommand ledDefaultCommand = new LEDDefaultCommand(this,m_led);

        // Set Default Commands
        m_drive.setDefaultCommand(driveDefaultCommand);
        m_arm.setDefaultCommand(armDefaultCommand);
        m_drone.setDefaultCommand(droneDefaultCommand);
        m_led.setDefaultCommand(ledDefaultCommand);

        // Set up buttons Driver
        // Left/Right Thumbstick for driving done in the DriveDefaultCommand class
        // The autonomous play will determine if the team is on RED or BLUE and set a global variable to use for angles.
        // Reset Gyro
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> m_drive.resetGyro(), m_drive));
        // Toggle is Field Oriented Mode
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> m_drive.toggleIsFieldOriented(), m_drive));

        // Set Arm Position Horizontal
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> m_arm.setArmPosition(ArmPos.STRAIGHT), m_arm));

        // Set Drive PID to -45
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> m_drive.setDrivePIDAngle(-45), m_drive));
        // Set Drive PID to 45 for wing Red
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> m_drive.setDrivePIDAngle(45), m_drive));

        // Set Drive PID to 90
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> m_drive.setDrivePIDAngle(90), m_drive));

        // Set Drive PID to -90
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> m_drive.setDrivePIDAngle(-90), m_drive));

        // Set up buttons for Operator

        // Close the claw (left Bumper)
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()-> m_arm.setClawGripAngle(k.CLAW.CloseAngle),m_arm));
        // Close the claw (right Bumper)
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> m_arm.setClawGripAngle(k.CLAW.OpenAngle),m_arm));
        // Lower Arm to floor (Button X(ps)/A(xbox))
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> m_arm.setArmPosition(ArmPos.FLOOR), m_arm));
        // Raise Arm to be vertical for climbing (Button Triangle(ps)/Y(xbox))
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> m_arm.setArmPosition(ArmPos.STACK_5), m_arm));
        // Extend and Retract forearm (Left X and Y Axis)

        // Set arm to horizontal (Button Square(ps)/X(xbox))
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> m_arm.setArmPosition(ArmPos.STRAIGHT), m_arm));
        // Set arm to backdrop (Button Circle(ps)/B(xbox))
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> m_arm.setArmPosition(ArmPos.STACK_3), m_arm));
        // Rotate Arm (Right Y Axis)
        // Launch Drone (Button back)
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new DroneLaunchCommand(this,m_drone));
        // Set Lower Color (White DPAD.UP, Green DPAD.DOWN, Purple DPAD.LEFT, Yellow DPAD.RIGHT)
        // Set Upper Color Right Bumper and (White DPAD.UP, Green DPAD.DOWN, Purple DPAD.LEFT, Yellow DPAD.RIGHT)



        m_timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            // Calculate the run rate of this loop
            // TODO: Handle the LEDs here since there is no subsystem for the LEDs

            telemetry.update();
            telemetry.addData("CPU Load TeleOp %", 100 - m_timer.remainingTime());
            // wait till timer is > 100ms to try an create a stable run rate
            while(!m_timer.done()){} m_timer.start();
        }
        reset();
    }
}
