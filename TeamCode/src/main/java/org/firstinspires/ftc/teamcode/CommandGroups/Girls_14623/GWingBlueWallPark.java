package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmAutoGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGetTeamPropLocation;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoResetGyroCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingBlueWallPark extends SequentialCommandGroup {

    public GWingBlueWallPark(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        ArmData armData = new ArmData();
        addCommands(
                new InstantCommand(_drive::resetYaw),                                      // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),                             // Raise the arm
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.6, 0,1.75),                          // Drive to team prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),             // Check the center
                new AutoRotateRobot(_opMode,_drive, -55,0.25,3),                                // Rotate to the left
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),              // Check the left
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),               // Check the right
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new ArmAutoGotoPosition(_opMode, _arm, 20, -12, 0),                             // Lower the arm
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),        // Release the lower pixel
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let it drop
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),               // Close the claw
                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),                             // Set arm to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate back to center
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.6, 0,1.65),                        // Backup
                new AutoRotateRobot(_opMode,_drive, -90,0.25,3),                                // Rotate to go under the truss
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.8, -90,2.05),                      // Drive under the truss
                new AutoRotateRobot(_opMode,_drive, 90,0.25,3),                                 // Rotate to the backdrop
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.4, 90,3.05),                       // Drive to the wall
                new ClawRotateFingers(_opMode, _claw, _claw.getClawOpenAngle()),                // Open the claw
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.4, 90,.6),                          // Backup a little
                new ArmAutoGotoPosition(_opMode, _arm, 0, 0, 0),                                // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the arm drop
                new AutoStopOpModeCommand(_opMode)                                              // This must be the last line of every command list
        );

    }
}
