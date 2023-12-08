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
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoResetGyroCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingBlueWallToBackdrop extends SequentialCommandGroup {

    public GWingBlueWallToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        ArmData armData = new ArmData();
        addCommands(
                new AutoResetGyroCommand(_opMode, _drive),                                      // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),                             // Raise the arm
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.6, 0,1.75),                          // Drive to team prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),             // Check the center
                new AutoRotateRobot(_opMode,_drive, -55,0.25,3),                                 // Rotate to the left
                new AutoDelayCommand(_opMode, .75),
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),               // Check the left
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),              // Check the right
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new ArmAutoGotoPosition(_opMode, _arm, 20, -12, 0),                             // Lower the arm
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),        // Release the lower pixel
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the pixel fall
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),               // Close the claw
                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),                             // Set arm to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate to the center
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.6, 0,1.65),                        // Drive back to the wall
                new AutoRotateRobot(_opMode,_drive, -90,0.25,3),                                 // Rotate to go under truss
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.8, -90,2.05),                        // Drive under truss
                new ArmAutoGotoPosition(_opMode, _arm, 42, 30, 0),                              // Raise the arm to the backdrop
                new AutoDriveToBackdropFromWing(_opMode,_drive),                                // Drive to the backdrop
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseUpperAngle()),        // Release the upper pixel
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.5, -90,0.6),                       // Drive back a little
                new AutoDriveToPark(_opMode, _drive, Direction.LEFT),                          // Park to the right
                new ArmAutoGotoPosition(_opMode, _arm, 0, 0, 0),                                // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay so the arm comes down
                new AutoStopOpModeCommand(_opMode)                                              // This must be the last line of every command list


        );

    }
}
