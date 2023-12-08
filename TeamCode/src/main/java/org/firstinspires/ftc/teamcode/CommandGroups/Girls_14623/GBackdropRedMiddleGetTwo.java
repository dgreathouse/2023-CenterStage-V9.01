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
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdropFromWingMiddle;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToMiddleFromBackdrop;
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

public class GBackdropRedMiddleGetTwo extends SequentialCommandGroup {

    public GBackdropRedMiddleGetTwo(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        ArmData armData = new ArmData();
        addCommands(
                new AutoResetGyroCommand(_opMode, _drive),                                      // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),                             // Raise Arm and lower claw
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.5, 0,1.65, 0.5, 0.5),                // Drive to team prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),             // Check the center
                new AutoRotateRobot(_opMode,_drive, -75,0.25,3),                                // Rotate to the right
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),              // Check the Right
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),               // Check the Left
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new ArmAutoGotoPosition(_opMode, _arm, 20, -12, 0),                             // Lower arm to drop the pixel
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),        // Drop the lower pixel
                new AutoDelayCommand(_opMode, 1),                                               // Delay so the pixel can drop
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),               // Close the claw so it does not come out
                new ArmAutoGotoPosition(_opMode, _arm, 35, 20, 0),                              // Raise the arm and set claw angle to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate robot back to center
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.6, 0,1.55),                        // Drive back away from team prop
                new AutoDriveToBackdrop(_opMode,_drive),                                        // Drive to the backdrop
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseUpperAngle()),        // Release the upper pixel
                new AutoDelayCommand(_opMode, .75),                                             // Delay so the pixel can drop
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, -90,.5),                        // Drive back a little
                new AutoDriveToMiddleFromBackdrop(_opMode,_drive),                              // Drive to the middle of the field facing the audience
                new ClawRotateFingers(_opMode, _claw, _claw.getClawOpenAngle()),                // Open the claw to be ready to grab them
                new ArmAutoGotoPosition(_opMode, _arm, 35, -6, 0),                              // Raise the arm and set claw angle lower to get a pixel
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, 90,2.75,.75,.75),               // Drive under truss
                new ArmAutoGotoPosition(_opMode, _arm, 7, -6, 0),                               // Lower arm to grab pixels
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.25, 90,1.20, 0, 0.75),             // Drive to pixel
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),               // Grab the pixels
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.7, 90,1, 1, 0),                     // Drive back a little
                new ArmAutoGotoPosition(_opMode, _arm, 35, -6, 0),                              // Raise the arm
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.7, 90,1.50, 0, 0),                  // Drive under truss
                new ArmAutoGotoPosition(_opMode, _arm, 5, -6, 0),                               // Set arm to floor
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.5, -90,1.8, 0, 1),                  // Drive to backstage
                new ArmAutoGotoPosition(_opMode, _arm, 0, 0, 0),                                // Lower the arm to the floor
                new ClawRotateFingers(_opMode, _claw, _claw.getClawOpenAngle()),                // Open claw
                new AutoDelayCommand(_opMode, 2),                                               // Delay to let the arm lower
                new AutoStopOpModeCommand(_opMode)                                              // This must be the last line of every command list

        );

    }
}
