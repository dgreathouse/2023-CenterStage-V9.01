package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.DriveToBackdrop;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.MiddleGetTwoCommand;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.StartCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToMiddleFromBackdrop;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropMiddleGetTwo extends SequentialCommandGroup {

    /**
     *
     * @param _opMode
     * @param _drive
     * @param _arm
     * @param _claw
     */
    public GBackdropMiddleGetTwo(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(
                new StartCommand(_opMode,_drive,_arm,_claw),                            // Drop the purple pixel and backup to wall
                new DriveToBackdrop(_opMode,_drive,_arm,_claw),                         // Go to the backdrop from the wall
                new MiddleGetTwoCommand(_opMode,_drive, _arm, _claw)                    // Get two from opposite wall from backdrop
        );

    }
}
