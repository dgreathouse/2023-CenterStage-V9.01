package org.firstinspires.ftc.teamcode.CommandGroups.AutoBackdrop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGetTeamPropLocation;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.AutoDetectAprilTag;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToTeamProp;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class AutoBackdropBlue_Play_1 extends SequentialCommandGroup {

    public AutoBackdropBlue_Play_1(CommandOpMode _opMode, DriveSubsystem _drive, ArmSubsystem _arm) {
        GlobalData.s_teamColor = TeamColor.BLUE;
        addCommands(
                new ArmGetTeamPropLocation(_opMode, _arm),
                new AutoDriveToTeamProp(_opMode, _drive),
                new AutoRotateRobot(_opMode, _drive, 90, 0.5, 2),
                new ArmGotoPosition(_opMode, _arm, ArmPos.ANGLE_30),
                new ParallelCommandGroup(
                        new AutoDriveToBackdrop(_opMode, _drive)

                ),
                new ArmRotateFingers(_opMode,_arm, k.CLAW.OpenUpperAngle),
                new AutoDriveTimeVel(_opMode, _drive, 90,0.5, -90,2000),
                new ArmGotoPosition(_opMode,_arm,ArmPos.FLOOR)
        );

    }
}
