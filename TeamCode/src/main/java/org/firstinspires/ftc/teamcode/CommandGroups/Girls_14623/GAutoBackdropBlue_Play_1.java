package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GAutoBackdropBlue_Play_1 extends SequentialCommandGroup {

    public GAutoBackdropBlue_Play_1(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        ArmData armData = new ArmData();
        addCommands(
                new AutoDriveTimeVel(_opMode,_drive,0.0,0.8,0.0,1.2, .85, 0.2),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),                   // Read team prop location (center)
//                new AutoRotateRobot(_opMode,_drive, 50,0.25,3000),                                    // Rotate to other team prop location
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),                     // Read team prop location (left)
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),                    // Read team prop location (right), just calculates the final position

                new AutoStopOpModeCommand(_opMode)                                                    // Stop the opMode
        );
    }
}
