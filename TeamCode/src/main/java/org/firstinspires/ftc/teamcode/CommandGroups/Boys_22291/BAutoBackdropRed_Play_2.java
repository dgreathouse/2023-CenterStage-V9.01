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
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToTeamProp;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class BAutoBackdropRed_Play_2 extends SequentialCommandGroup {

    public BAutoBackdropRed_Play_2(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        ArmData armData = new ArmData();
        addCommands(
//
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),
//                new AutoDelayCommand(_opMode,1000),
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0),
//                new AutoDriveTimeVel(_opMode, _drive,0,0.4,0,1400,true),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),
//                new AutoRotateRobot(_opMode,_drive, -45,0.25,3000),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),
//                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_3),0,0),
//                new AutoDriveToTeamProp(_opMode,_drive),
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),
//
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0),
//                new AutoDriveAwayFromTeamProp(_opMode, _drive),
//                new AutoDriveToBackdrop(_opMode,_drive),
//                new ClawRotateFingers(_opMode,_claw, _claw.getClawOpenAngle()),
//                new AutoDriveTimeVel(_opMode, _drive,-90,0.3,-90,550,true),
//                new AutoDriveTimeVel(_opMode, _drive,-180,0.3,-90,1700,false),                       // Drive to the left and park
//                new ArmAutoGotoPosition(_opMode,_arm,armData.getArmSetAngle(ArmPos.FLOOR),0,0),        // Put the arm Straight
//                new AutoDelayCommand(_opMode,1000),
//                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list

        );

    }
}
