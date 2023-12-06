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
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdropFromWing;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdropFromWingMiddle;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveWithDistanceSensor;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoResetGyroCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropRedParkMiddle extends SequentialCommandGroup {

    public GBackdropRedParkMiddle(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        ArmData armData = new ArmData();
        addCommands(
//                new AutoResetGyroCommand(_opMode, _drive),                                      // Reset the gyro
//                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
//                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),                             // Raise Arm and lower claw
//                new AutoDriveTimeVel(_opMode, _drive, 0, 0.6, 0,1.75),                          // Drive to team prop
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),             // Check the center
//                new AutoRotateRobot(_opMode,_drive, -75,0.25,3),                                // Rotate to the right
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),              // Check the Right
//                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),               // Check the Left
//                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
//                new ArmAutoGotoPosition(_opMode, _arm, 20, -12, 0),                             // Lower arm to drop the pixel
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),        // Drop the lower pixel
//                new AutoDelayCommand(_opMode, 1),                                               // Delay so the pixel can drop
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),               // Close the claw so it does not come out
//                new ArmAutoGotoPosition(_opMode, _arm, 42, 30, 0),                              // Raise the arm and set claw angle to backdrop
//                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate robot back to center
//                new AutoDriveTimeVel(_opMode, _drive, 180, 0.6, 0,1.65),                        // Drive back away from team prop
//                new AutoDriveToBackdrop(_opMode,_drive),                                        // Drive to the backdrop
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseUpperAngle()),        // Release the upper pixel
//                new AutoDelayCommand(_opMode, .75),                                             // Delay so the pixel can drop
//                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, -90,0.6),                       // Drive backwards away from backdrop so it can move
//                new AutoDriveToPark(_opMode, _drive, Direction.LEFT),                           // Park on the left
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawOpenAngle()),                // Open the claw to be ready to grab them
//                new ArmAutoGotoPosition(_opMode, _arm, 35, -6, 0),                              // Raise the arm and set claw angle to backdrop
//                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,2.5),                       // Drive under truss
//                new AutoRotateRobot(_opMode,_drive, 90,0.25,3),                                 // Rotate robot
//                new AutoDriveWithDistanceSensor(_opMode,_drive, -90, 0.27, 90, 200, 3),         // Drive to the stack of pixels
//                new ArmAutoGotoPosition(_opMode, _arm, 6, -6, 0),                               // Lower arm to grab pixels
//                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the arm lower
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),               // Grab the pixels
//                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,1, 1, 0),                   // Drive back a little
//                new ArmAutoGotoPosition(_opMode, _arm, 35, -6, 0),                              // Raise the arm
//                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,2.5, 0, 1),                 // Drive under truss
//                new ArmAutoGotoPosition(_opMode, _arm, 48, 30, 0),                              // Set arm to backdrop
//                new AutoDriveToBackdropFromWingMiddle(_opMode,_drive),                          // Drive to backdrop
//                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseUpperAngle()),        // Release upper, hopefully two will fall out nicely if we slowly backup
//                new AutoDriveToPark(_opMode, _drive, Direction.LEFT),                           // Park on the left
//                new ArmAutoGotoPosition(_opMode, _arm, 0, 0, 0),                                // Lower the arm to the floor
//                new AutoDelayCommand(_opMode, 2),                                               // Delay to let the arm lower
//                new AutoStopOpModeCommand(_opMode)                                              // This must be the last line of every command list
        );

    }
}
