package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class GAutoWingRed_Play_5 extends SequentialCommandGroup {

    public GAutoWingRed_Play_5(CommandOpMode _opMode, DriveSubsystem _drive, ArmSubsystem _arm) {
        GlobalData.TeamColor = TeamColor.RED;
        addCommands(

                //new AutoDelayCommand(_opMode,10000),
                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}
