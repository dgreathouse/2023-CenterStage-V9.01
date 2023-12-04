package org.firstinspires.ftc.teamcode.CommandGroups.Boys_22291;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class BWingRedWallToBackdrop extends SequentialCommandGroup {

    public BWingRedWallToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        ArmData armData = new ArmData();
        addCommands(
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),
//                new AutoDelayCommand(_opMode,1000),
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0),
//                new AutoDriveTimeVel(_opMode, _drive,0,0.4,0,1400,true),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),
//                new AutoRotateRobot(_opMode,_drive, 45,0.25,3000),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),
//                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.FLOOR),0,0),
//                new AutoDriveToTeamProp(_opMode,_drive),
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),
//
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0),
//                new AutoRotateRobot(_opMode, _drive, 0, 0.25, 3000),
//                new AutoDriveAwayFromTeamProp(_opMode, _drive),
//
//                new ArmAutoGotoPosition(_opMode,_arm,armData.getArmSetAngle(ArmPos.FLOOR),0,0),        // Put the arm Straight
//                new AutoDelayCommand(_opMode,1000),


                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}
