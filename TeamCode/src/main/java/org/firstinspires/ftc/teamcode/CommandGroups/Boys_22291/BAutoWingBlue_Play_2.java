package org.firstinspires.ftc.teamcode.CommandGroups.Boys_22291;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.ArmAutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawAutoGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class BAutoWingBlue_Play_2 extends SequentialCommandGroup {

    public BAutoWingBlue_Play_2(CommandOpMode _opMode, DriveSubsystem _drive, ArmAutoSubsystem _arm, ClawAutoGripSubsystem _claw) {
        GlobalData.TeamColor = TeamColor.BLUE;
        GlobalData.FieldLocation = AutoFieldLocation.WING;
        addCommands(


                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}
