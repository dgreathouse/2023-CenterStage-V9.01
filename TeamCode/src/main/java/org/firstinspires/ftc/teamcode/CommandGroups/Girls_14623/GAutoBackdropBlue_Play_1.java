package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

/** Girls Auto Backdrop Blue Play 1
 * Detect team prop and drop off pixel at spike mark.
 * Drive to backdrop and drop off pixel.
 * Drive to the right to park our of the way for the backdrop.
 */
public class GAutoBackdropBlue_Play_1 extends SequentialCommandGroup {

    public GAutoBackdropBlue_Play_1(CommandOpMode _opMode, DriveSubsystem _drive, ArmSubsystem _arm) {
        GlobalData.TeamColor = TeamColor.BLUE;
        addCommands(

                new AutoStopOpModeCommand(_opMode)
        );

    }
}
