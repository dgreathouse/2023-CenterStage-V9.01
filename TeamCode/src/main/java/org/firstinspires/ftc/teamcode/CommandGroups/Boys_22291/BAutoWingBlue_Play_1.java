package org.firstinspires.ftc.teamcode.CommandGroups.Boys_22291;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmAutoGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGetTeamPropLocation;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveAwayFromTeamProp;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToTeamProp;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.ArmAutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawAutoGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class BAutoWingBlue_Play_1 extends SequentialCommandGroup {

    public BAutoWingBlue_Play_1(CommandOpMode _opMode, DriveSubsystem _drive, ArmAutoSubsystem _arm, ClawAutoGripSubsystem _claw) {
        GlobalData.TeamColor = TeamColor.BLUE;
        GlobalData.FieldLocation = AutoFieldLocation.WING;
        ArmData armData = new ArmData();
        addCommands(
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),
                new AutoDelayCommand(_opMode,1000),
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT)),
                new AutoDriveTimeVel(_opMode, _drive,0,0.4,0,1400),
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),
                new AutoRotateRobot(_opMode,_drive, -45,0.25,3000),                                  // sign
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_3)),
                new AutoDriveToTeamProp(_opMode,_drive),
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),

                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT)),
                new AutoDriveAwayFromTeamProp(_opMode, _drive),

                new ArmAutoGotoPosition(_opMode,_arm,armData.getArmSetAngle(ArmPos.FLOOR)),
                new AutoDelayCommand(_opMode,1000),


                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}
