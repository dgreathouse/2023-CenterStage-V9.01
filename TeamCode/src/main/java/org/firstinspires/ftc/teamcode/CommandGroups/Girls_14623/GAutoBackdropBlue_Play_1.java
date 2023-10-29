package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGetTeamPropLocation;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToTeamProp;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

/** Girls Auto Backdrop Blue Play 1
 * Detect team prop and drop off pixel at spike mark.
 * Drive to backdrop and drop off pixel.
 */
public class GAutoBackdropBlue_Play_1 extends SequentialCommandGroup {

    public GAutoBackdropBlue_Play_1(CommandOpMode _opMode, DriveSubsystem _drive, ArmSubsystem _arm) {
        GlobalData.TeamColor = TeamColor.BLUE;
        addCommands(
                // TODO: This was copied from "GAutoBackdropRed_Play_1" Update this for being on the BLUE side
                // BEGIN Initial Backdrop Pixels
                new ArmRotateFingers(_opMode,_arm, _arm.getClawCloseAngle()),               // Grab the pixels
                new AutoDelayCommand(_opMode,500),                                          // Delay while the claw grabs the pixels
                new ArmGotoPosition(_opMode,_arm,_arm.getArmSetAngle(ArmPos.STACK_3)),      // Raise the arm to Stack of 3 to drive
                new AutoDriveTimeVel(_opMode, _drive,0,0.4,0,1200),                         // Drive out to be closer to Team Prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),         // Check the Center Team Prop
                new AutoRotateRobot(_opMode,_drive, 45,0.25,3000),                         // Turn to the other Team Prop location
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),          // Check the Right Team Prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),           // Check the Right Team Prop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),                           // Rotate to be straight again
                new AutoDriveToTeamProp(_opMode,_drive),                                    // Drive to team prop based on location
                new ArmRotateFingers(_opMode,_arm, _arm.getClawReleaseLowerAngle()),        // Release Lower Pixel
                new ArmGotoPosition(_opMode,_arm,_arm.getArmSetAngle(ArmPos.STRAIGHT)),     // Put the arm Straight
                new AutoDriveToBackdrop(_opMode,_drive),                                    // Drive to the backdrop
                new ArmRotateFingers(_opMode,_arm, _arm.getClawReleaseLowerAngle()),        // Release upper claw
                new AutoDriveTimeVel(_opMode, _drive,90,0.3,90,500),                      // Drive back away from the backdrop
                // END Initial Backdrop Pixels
                new AutoDriveTimeVel(_opMode, _drive,0,0.3,90,1500),                       // Drive to the left and park

                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list
        );

    }
}
