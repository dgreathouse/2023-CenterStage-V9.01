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
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoResetGyroCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropRedWallGetTwo extends SequentialCommandGroup {

    public GBackdropRedWallGetTwo(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        ArmData armData = new ArmData();
        addCommands(
                new AutoResetGyroCommand(_opMode, _drive),                                      // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),                             // Raise Arm and lower claw
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.5, 0,1.65, 0.5, 0.5),                  // Drive to team prop
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
                new AutoDelayCommand(_opMode, .75),
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, -90,.75),                       // Delay so the pixel can drop
                new AutoDriveToPark(_opMode, _drive, Direction.RIGHT),
                new ArmAutoGotoPosition(_opMode, _arm, 35, -6, 0),                              // Raise the arm and set claw angle to backdrop
                new ClawRotateFingers(_opMode, _claw, _claw.getClawOpenAngle()),                // Release the upper pixel
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,3.35),                        // Drive back away from team prop
                new AutoRotateRobot(_opMode,_drive, 90,0.25,3),                                  // Rotate robot back to center
                new ArmAutoGotoPosition(_opMode, _arm, 8, -6, 0),                              // Raise the arm and set claw angle to backdrop
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.7, 90,1.68),                        // Drive back away from team prop
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.25, 90,.55,0,0),                        // Drive back away from team prop
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),                // Release the upper pixel
                new AutoDelayCommand(_opMode, .75),
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.25, 90,.3,0,0),
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.7, 90,1.68),                        // Drive back away from team prop
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.7, 90,1,.75,0),
                new ArmAutoGotoPosition(_opMode, _arm,35, -6, 0),
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.7, 90,1.2,0,0),                        // Drive back away from team prop
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.5, -90,1.4,0,.5),                        // Drive back away from team prop

                new ClawRotateFingers(_opMode, _claw, _claw.getClawOpenAngle()),                // Release the upper pixel
                new ArmAutoGotoPosition(_opMode, _arm, 0, 0, 0),                              // Raise the arm and set claw angle to backdrop
                new AutoDelayCommand(_opMode, 1),
                new AutoStopOpModeCommand(_opMode)                                              // This must be the last line of every command list

        );

    }
}
