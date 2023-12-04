package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmAutoGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GAutoBackdropBlueParkMiddle extends SequentialCommandGroup {

    public GAutoBackdropBlueParkMiddle(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        ArmData armData = new ArmData();
        addCommands(
                new AutoDelayCommand(_opMode, 1.0),                                                         // Wait for claw to close
                new ArmAutoGotoPosition(_opMode,_arm,35.0,0.0, 0.0),          // Raise the arm
                new AutoDelayCommand(_opMode, 0.5),                                             // Wait for arm to raise before driving
                new AutoDriveTimeVel(_opMode, _drive, 0, 1.0, 0, 2, 0,1),       // Drive out closer to the Team Propnew AutoDelayCommand(_opMode, 10.0),
//                new AutoRotateRobot(_opMode,_drive,-90, 0.3,2),
//                new AutoDriveTimeVel(_opMode, _drive, -90, .80, -90, 3, 1.0,0.5),       // Drive out closer to the Team Propnew AutoDelayCommand(_opMode, 10.0),
                new AutoDelayCommand(_opMode, 5),
                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}
