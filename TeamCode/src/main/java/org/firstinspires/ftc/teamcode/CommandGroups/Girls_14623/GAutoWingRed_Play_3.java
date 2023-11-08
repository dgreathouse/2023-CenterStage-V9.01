package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

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
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;

public class GAutoWingRed_Play_3 extends SequentialCommandGroup {

    public GAutoWingRed_Play_3(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {
        GlobalData.TeamColor = TeamColor.RED;
        GlobalData.FieldLocation = AutoFieldLocation.WING;
        ArmData armData = new ArmData();
        addCommands(
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),
                new AutoDelayCommand(_opMode,1000),
                new ArmAutoGotoPosition(_opMode, _arm, 35,0,0),
                new AutoDriveTimeVel(_opMode, _drive,0,0.4,0,1500), //drive to team prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),
                new AutoRotateRobot(_opMode,_drive, 45,0.25,3000),  //turn to left
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),  //face forward
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_3),0,0),
                new AutoDriveToTeamProp(_opMode,_drive),
                new ArmAutoGotoPosition(_opMode, _arm, 25,-5,80),
                new AutoDelayCommand(_opMode,2000),
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),
                new AutoDelayCommand(_opMode,1000),
                new ArmAutoGotoPosition(_opMode, _arm, 25,30,00),
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),
                new AutoDriveTimeVel(_opMode, _drive,180,0.4,0,1300),  //goto wall
                new AutoRotateRobot(_opMode,_drive, -90,0.25,3000),  //face backdrop
                new AutoDriveTimeVel(_opMode, _drive,90,0.4,-90,3000),  //drive under truss
                new AutoDriveTimeVel(_opMode, _drive,60,0.4,-90,2000),  // drive to backdrop
//
//                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0),
//                new AutoDriveAwayFromTeamProp(_opMode, _drive),
//
//                new AutoDriveTimeVel(_opMode, _drive,-180,0.4,-90,800),    //drive and rotate back to the wall
//                new AutoDriveTimeVel(_opMode, _drive,90,0.4,-90,5000),    //drive under truss
//                new AutoDriveTimeVel(_opMode, _drive,-45,0.4,-90,5000),   //drive at angle to backdrop
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseUpperAngle()),
//                new AutoDriveTimeVel(_opMode, _drive,-90,0.4,-90,800),   //back away from backdrop
//                new ArmAutoGotoPosition(_opMode,_arm,armData.getArmSetAngle(ArmPos.FLOOR),0,0),        // Put the arm Straight
//                new AutoDriveTimeVel(_opMode, _drive,0,0.4,-90,800),    //drive to center
                new AutoDelayCommand(_opMode,10000),
                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list

        );

    }
}
