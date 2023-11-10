package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmAutoGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGetTeamPropLocation;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveAwayFromTeamProp;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
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

public class GAutoBackdropRed_Play_1 extends SequentialCommandGroup {

    public GAutoBackdropRed_Play_1(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {
        GlobalData.TeamColor = TeamColor.RED;
        GlobalData.FieldLocation = AutoFieldLocation.BACKDROP;
        ArmData armData = new ArmData();
        addCommands(
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),                   //close claw
                new AutoDelayCommand(_opMode,1000),                                                 //delay
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0),//arm go straight
                new AutoDriveTimeVel(_opMode, _drive,0,0.4,0,1400),                                 //drive up to spike mark
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),                 //read team prop location (center)
                new AutoRotateRobot(_opMode,_drive, -50,0.25,3000),                                 //rotate right
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),                  //check if team prop is on right
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),                   //check if team prop is on left
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),                                   //turn straight
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_5),0,0), //put arm in position
                new AutoDriveToTeamProp(_opMode,_drive),                                            //drive to team prop
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_5),0,100),
                new AutoDelayCommand(_opMode,1250),//put arm in position
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),20,0),
                new AutoDelayCommand(_opMode,250),//put arm in position//
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),
                new AutoDriveTimeVel(_opMode, _drive,-180,0.4,0,800),
                new AutoDriveToBackdrop(_opMode,_drive),
                new ClawRotateFingers(_opMode,_claw, _claw.getClawOpenAngle()),
                new AutoDriveTimeVel(_opMode, _drive,-90,0.3,-90,550),
                new AutoDriveToPark(_opMode, _drive, Direction.LEFT), // Drive to the left and park
                new ArmAutoGotoPosition(_opMode,_arm,armData.getArmSetAngle(ArmPos.FLOOR),0,0),        // Put the arm Straight
                new AutoDelayCommand(_opMode,1000),
                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list


        );

    }
}
