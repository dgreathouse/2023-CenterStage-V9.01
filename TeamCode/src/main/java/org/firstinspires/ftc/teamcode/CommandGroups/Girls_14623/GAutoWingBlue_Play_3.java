package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmAutoGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGetTeamPropLocation;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdropFromWing;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToTeamProp;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GAutoWingBlue_Play_3 extends SequentialCommandGroup {

    public GAutoWingBlue_Play_3(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        ArmData armData = new ArmData();
        addCommands(
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),                     // Close claw to grab pixels
                new AutoDelayCommand(_opMode,1000),                                                   // Delay to let claw close on the pixels//delay
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0),  // Raise arm straight so distance sensor can see team prop
                new AutoDriveTimeVel(_opMode, _drive,0,0.4,0,1400),                                   // Drive up to team prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),                   // Read team prop location (center)//read team prop location (center)
                new AutoRotateRobot(_opMode,_drive, -50,0.25,3000),                                    // Rotate to other team prop location//rotate right
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),                     // Read team prop location (left)
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),                    // Read team prop location (right), just calculates the final position
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),                                     // Rotate robot straight
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_5),0,0),   // Put arm in position to drop pixel
                new AutoDriveToTeamProp(_opMode,_drive),                                              // Drive to team prop method that just rotates to a location
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_5),0,62),  // Extend arm to spike mark
                new AutoDelayCommand(_opMode,1250),                                                   // Delay long enough for arm to reach spike mark
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),              // Release the lower pixel only
                new AutoDelayCommand(_opMode,250),                                                    // Delay to allow the claw to release instead of flicking it away
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),20,0), // Raise arm and tilt the claw and retract the forearm
                new AutoDelayCommand(_opMode,250),                                                    // Delay for arm to go straight so the rotation does not hit the team prop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),                                     // Rotate robot straight
                new AutoDriveTimeVel(_opMode, _drive,-180,0.4,0,1150),                                // Drive backwards to a the wall
                new AutoRotateRobot(_opMode,_drive, -90,0.25,3500),                                    // Rotate so back is towards the backdrop
                new AutoDriveTimeVel(_opMode, _drive,-90,0.4,-90,2800),                                 // Drive under truss
                new AutoDelayCommand(_opMode,100),                                                    // Delay to let other robot move out of the way. Adjust for other robot
                new AutoDriveToBackdropFromWing(_opMode,_drive),                                      // Drive the correct position on the backdrop
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseUpperAngle()),              // Release the upper pixel
                new AutoDriveTimeVel(_opMode, _drive,90,0.3,90,550),                                // Drive backwards away from backdrop
                new AutoDriveToPark(_opMode, _drive, Direction.RIGHT),                                 // Drive to the left and park
                new ArmAutoGotoPosition(_opMode,_arm,armData.getArmSetAngle(ArmPos.FLOOR),0,0),       // Put the arm to the floor
                new AutoDelayCommand(_opMode,1000),                                                   // Delay for arm to reach floor
                new AutoStopOpModeCommand(_opMode)


        );

    }
}
