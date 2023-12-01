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
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GAutoBackdropRedParkMiddleGetTwo extends SequentialCommandGroup {

    public GAutoBackdropRedParkMiddleGetTwo(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        ArmData armData = new ArmData();
        addCommands(
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),                                           // Close claw to grab pixels
                new AutoDelayCommand(_opMode,1.0),                                                                 // Delay to let claw close on the pixels
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),0,0), // Raise arm straight so distance sensor can see team prop
                new AutoDriveTimeVel(_opMode, _drive,0,0.4,0,1.4,1,.4),                                   // Drive up to team prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),                                          // Read team prop location (center)
                new AutoRotateRobot(_opMode,_drive, -50,0.25,3.0),                                    // Rotate to other team prop location
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),                                           // Read team prop location (right)
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),                                            // Read team prop location (left), just calculates the final position
                //new AutoRotateRobot(_opMode,_drive, 0,0.25,3000),                                      // Rotate robot straight
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_5),0,0),   // Put arm in position to drop pixel
                new AutoRotateToTeamProp(_opMode,_drive),                                                                     // Drive to team prop method that just rotates to a location
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_5),0,100), // Extend arm to spike mark
                new AutoDelayCommand(_opMode,1.25),                                                                  // Delay long enough for arm to reach spike mark
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),                                     // Release the lower pixel only
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),20,0), // Raise arm and tilt the claw
                new AutoDelayCommand(_opMode,0.25),                                                                  // Delay for pixel to drop out of the claw
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3.0),                                     // Rotate robot straight
                new AutoDriveTimeVel(_opMode, _drive,-180,0.4,0,0.8,0.5,0.3),                   // Drive backwards to a known point
                new AutoDriveToBackdrop(_opMode,_drive),                                                                    // Drive to backdrop method
                new ClawRotateFingers(_opMode,_claw, _claw.getClawOpenAngle()),                                             // At the back drop now so drop the upper pixel
                new AutoDriveTimeVel(_opMode, _drive,-90,0.3,-90,0.55,0.2,0.1),                   // Drive backwards away from backdrop
                new AutoDriveToPark(_opMode, _drive, Direction.RIGHT),                                                      // Drive to the left and park
                new AutoDriveTimeVel(_opMode, _drive,-90,0.3,-90,3.0,1.0,1.0),                  // Drive to stack of pixels backwards
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STACK_5),-6,0), // Lower arm and tilt the claw for top two pixels
                new AutoDriveTimeVel(_opMode, _drive,-90,0.3,90,3.0,1.0,1.0),                   // Drive to stack of pixels forwards
                new ClawRotateFingers(_opMode,_claw, _claw.getClawCloseAngle()),                                             // Grab the pixels
                new AutoDelayCommand(_opMode,1.25),                                                                  // Delay for claw to grab
                new ArmAutoGotoPosition(_opMode, _arm, armData.getArmSetAngle(ArmPos.STRAIGHT),20,0), // Raise arm and tilt the claw
                new AutoDriveTimeVel(_opMode, _drive,90,0.3,90,3.0,1.0,1.0),                    // Drive to backdrop backwards
                new AutoDriveTimeVel(_opMode, _drive,90,0.3,-90,3.0,1.0,1.0),                   // Drive to backdrop forwards
                new ClawRotateFingers(_opMode,_claw, _claw.getClawOpenAngle()),                                              // release the pixels
                new ArmAutoGotoPosition(_opMode,_arm,armData.getArmSetAngle(ArmPos.FLOOR),0,0),      // Put the arm to the floor
                new AutoDelayCommand(_opMode,1.0),                                                                 // Delay a second to allow arm to goto the floor before ending and turning off power
                new AutoStopOpModeCommand(_opMode)                                                                          // Stop the opMode

        );

    }
}
