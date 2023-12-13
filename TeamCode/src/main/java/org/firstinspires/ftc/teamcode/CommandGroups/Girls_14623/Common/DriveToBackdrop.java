package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class DriveToBackdrop extends SequentialCommandGroup {

    public DriveToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {
        double sign = GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE ? 1.0 : -1;
        addCommands(
                new AutoDriveToBackdrop(_opMode, _drive),
                new InstantCommand(_claw::setClawReleaseUpperAngle),                            // Release the upper pixel
                new AutoDelayCommand(_opMode, .75),                                             // Delay to let the pixel drop
                new AutoDriveTimeVel(_opMode, _drive, 90 * sign, 0.5, 90 * sign, .75)                          // Back up a little
        );

    }
}
