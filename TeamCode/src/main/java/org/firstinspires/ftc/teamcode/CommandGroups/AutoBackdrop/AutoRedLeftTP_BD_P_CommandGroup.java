package org.firstinspires.ftc.teamcode.CommandGroups.AutoBackdrop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGetTeamPropLocation;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToDistance;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class AutoRedLeftTP_BD_P_CommandGroup extends SequentialCommandGroup {

    public AutoRedLeftTP_BD_P_CommandGroup(CommandOpMode _opMode, DriveSubsystem _drive, ArmSubsystem _arm) {
        GlobalData.s_teamColor = TeamColor.RED;
        addCommands(

                new ArmGetTeamPropLocation(_opMode, _arm), // Detect team prop
                new AutoDriveToTeamProp(_opMode, _drive), // Drive to location for where prop is
               // new ArmClawLeftSpin(_opMode, _arm, -1, 1),  // Eject the correct pixel
                new ArmGotoPosition(_opMode, _arm, ArmPos.STRAIGHT), // Raise arm for backdrop
                new AutoDriveToBackdrop(_opMode, _drive),  // Drive to back drop by rotating, driving
              //  new ArmClawRightSpin(_opMode, _arm, -1, 1),
                new AutoDriveToDistance(_opMode, _drive, k.DRIVE.DriveDistance, k.DRIVE.DriveTime_Speed60, 5, 60, 90, 90), // Slide to corner
                new AutoDriveToDistance(_opMode, _drive, k.DRIVE.DriveDistance, k.DRIVE.DriveTime_Speed60, 2, 60, 0, 90),  // Slide in to corner to park
                new AutoStopOpModeCommand(_opMode)

        );

    }
}
