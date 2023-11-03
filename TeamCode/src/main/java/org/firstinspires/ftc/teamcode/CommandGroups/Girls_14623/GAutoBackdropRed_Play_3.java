package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.ArmAutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawAutoGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class GAutoBackdropRed_Play_3 extends SequentialCommandGroup {

    public GAutoBackdropRed_Play_3(CommandOpMode _opMode, DriveSubsystem _drive, ArmAutoSubsystem _arm, ClawAutoGripSubsystem _claw)  {
        GlobalData.TeamColor = TeamColor.RED;
        addCommands(

                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}
