package org.firstinspires.ftc.teamcode.CommandGroups.Boys_22291;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class BBackdropRedParkMiddle extends SequentialCommandGroup {

    public BBackdropRedParkMiddle(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        ArmData armData = new ArmData();

        addCommands(
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),
//                new AutoDelayCommand(_opMode,500),
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,100),
//                new AutoDriveTimeVel(_opMode, _drive,0,0.6,0,1400,true),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),-10,100),
//                new AutoRotateRobot(_opMode,_drive, -50,0.25,3000),// sign
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),
//                new AutoRotateToTeamProp(_opMode,_drive),
//                new ArmAutoGotoTeamProp(_opMode, _arm),
//                new AutoDelayCommand(_opMode, 1000),
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),
//
//                new AutoDelayCommand(_opMode,5000),
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0),
//                new AutoDelayCommand(_opMode, 5000),
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.FLOOR),0,0),
//                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}
