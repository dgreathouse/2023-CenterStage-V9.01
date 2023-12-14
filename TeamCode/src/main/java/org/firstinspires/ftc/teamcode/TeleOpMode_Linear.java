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
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawGripDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drone.DroneDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drone.DroneLaunchCommand;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.RobotState;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DroneSubsystem;

import java.util.concurrent.TimeUnit;


/**

 */

@TeleOp(name="TeleOp 1", group="Linear Opmode")

public class TeleOpMode_Linear extends CommandOpMode {
    Timing.Timer m_timer;

    private Hw hw;
    private DriveSubsystem m_drive;
    private ArmSubsystem m_arm;
    private DroneSubsystem m_drone;
    private ClawGripSubsystem m_clawGrip;

    @Override
    public void initialize() {
        hw = new Hw(this);
        hw.init();

        // Create subsystems
        m_drive = new DriveSubsystem(this, hw);
        m_arm = new ArmSubsystem(this);
        m_drone = new DroneSubsystem(this);
        m_clawGrip = new ClawGripSubsystem(this);
        // Create Default Commands
        DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(this, m_drive);
        ArmDefaultCommand armDefaultCommand = new ArmDefaultCommand(this,m_arm);
        DroneDefaultCommand droneDefaultCommand = new DroneDefaultCommand(this,m_drone);
        ClawGripDefaultCommand clawGripDefaultCommand = new ClawGripDefaultCommand(this, m_clawGrip);
        // Set Default Commands
        m_drive.setDefaultCommand(driveDefaultCommand);
        m_arm.setDefaultCommand(armDefaultCommand);
        m_drone.setDefaultCommand(droneDefaultCommand);
        m_clawGrip.setDefaultCommand(clawGripDefaultCommand);

        // Set up buttons Driver
        // Left/Right Thumbstick for driving done in the DriveDefaultCommand class
        // Reset Gyro
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> m_drive.resetGyro(), m_drive));
        // Toggle is Field Oriented Mode
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> m_drive.toggleIsFieldOriented(), m_drive));
        //(Button Circle(ps)/B(xbox))
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> m_arm.setArmAngle(ArmPos.STRAIGHT), m_arm));
        //  (Button X(ps)/A(xbox))
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> m_drive.setDriveSpeedScale(0.5), m_arm));
        //  (Button Triangle(ps)/Y(xbox))
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> m_drive.setDriveSpeedScale(1.0), m_arm));
        // (Button Square(ps)/X(xbox))
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> m_drive.toggleVeocityMode(), m_drive));
        // Set Drive PID to -45
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> m_drive.setDrivePIDAngle(135), m_drive));
        // Set Drive PID to 45 for wing Red
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> m_drive.setDrivePIDAngle(45), m_drive));
        // Set Drive PID to 90
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> m_drive.setDrivePIDAngle(90), m_drive));
        // Set Drive PID to -90
        Hw.s_gpDriver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> m_drive.setDrivePIDAngle(-90), m_drive));

        // Set up buttons for Operator
        // Close the claw (left Bumper)
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()-> m_clawGrip.setClawGripAngle(m_clawGrip.getClawOpenAngle()),m_arm));
        // Close the claw (right Bumper)
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()-> m_clawGrip.setClawGripAngle(m_clawGrip.getClawReleaseUpperAngle()),m_arm));
        // Lower Arm to floor (Button X(ps)/A(xbox))
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> m_arm.setArmAngle(ArmPos.FLOOR), m_arm));
        // Set arm to get pixels from top of stack of 5 (Button Triangle(ps)/Y(xbox))
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> m_arm.setArmAngle(ArmPos.ANGLE_BACKDROP), m_arm));
        // Set arm to Straight (Button Circle(ps)/B(xbox))
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> m_arm.setArmAngle(ArmPos.STRAIGHT), m_arm));
        // Set arm to Backdrop angle (Button Square(ps)/X(xbox))
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> m_arm.setArmAngle(ArmPos.STACK_5), m_arm));

        // Launch Drone (Button Left DPAD)
        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new DroneLaunchCommand(this,m_drone));

        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new InstantCommand(() -> m_arm.lowerClaw(-6), m_arm));
//        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new ArmIncreaseShouldUpVelocity(this,m_arm,0.01));
//        Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ArmIncreaseShouldDownVelocity(this,m_arm,-0.01));
        // Right DPAD is used in the ArmDefaultCommand to disable the arm movement so the forearm can be moved for climbing.
       // Hw.s_gpOperator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new));


        m_timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);

    }

    @Override
    public void runOpMode() throws InterruptedException{
        initialize();

        waitForStart();
        GlobalData.State = RobotState.DRIVERCONTROLLED;
        m_timer.start();
        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            // Calculate the run rate of this loop

            telemetry.update();
           // telemetry.addData("Match Time", m_timer.elapsedTime());
            // wait till timer is > 100ms to try an create a stable run rate
            if(k.SYSTEM.isLoopRateLimited){
                while(!m_timer.done()){} m_timer.start();
                telemetry.addData("CPU Load TeleOp %", 100 - m_timer.remainingTime());
            }
        }
        reset();
    }
}
