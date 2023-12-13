package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToMiddleFromBackdrop;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropMiddleGetTwo extends SequentialCommandGroup {

    public GBackdropMiddleGetTwo(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(
                new BackdropBlueStartCommand(_opMode,_drive,_arm,_claw),
                new AutoDriveToMiddleFromBackdrop(_opMode,_drive),                              // Drive to the middle of the field facing the audience
                new MiddleGetTwoCommand(_opMode,_drive, _arm, _claw)

        );

    }
}
