package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class GAutoBackdropBlue_Play_5 extends SequentialCommandGroup {

    public GAutoBackdropBlue_Play_5(CommandOpMode _opMode, DriveSubsystem _drive, ArmSubsystem _arm) {
        GlobalData.s_teamColor = TeamColor.BLUE;
        addCommands(
            new ParallelCommandGroup(

            )


        );

    }
}
