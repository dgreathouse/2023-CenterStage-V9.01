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

public class BAutoBackdropRed_Play_3 extends SequentialCommandGroup {

    public BAutoBackdropRed_Play_3(CommandOpMode _opMode, DriveSubsystem _drive, ArmAutoSubsystem _arm, ClawAutoGripSubsystem _claw) {
        GlobalData.TeamColor = TeamColor.RED;
        GlobalData.FieldLocation = AutoFieldLocation.BACKDROP;
        addCommands(

                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list



        );

    }
}
