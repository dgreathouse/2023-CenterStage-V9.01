package org.firstinspires.ftc.teamcode.CommandGroups.Boys_22291;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class BWingRedWallPark extends SequentialCommandGroup {

    public BWingRedWallPark(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(


                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}