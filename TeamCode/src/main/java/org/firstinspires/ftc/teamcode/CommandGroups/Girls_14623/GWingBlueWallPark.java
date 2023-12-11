package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToDistance;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingBlueWallPark extends SequentialCommandGroup {

    public GWingBlueWallPark(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(
                new InstantCommand(_drive::resetYaw),                                           // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new InstantCommand(() -> _arm.setArmData(35,-10,0)),                            // Raise the arm
                new AutoDriveToDistance(_opMode,_drive,620, 0.5, 0,0,3),                        // Drive to team prop
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.CENTER)),   // Check the center
                new AutoRotateRobot(_opMode,_drive, -55,0.25,3),                                // Rotate to the left
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.RIGHT)),    // Check the left
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.LEFT)),     // Check the right
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new InstantCommand(() -> _arm.setArmData(20,-12,0)),                            // Lower the arm
                new InstantCommand(_claw::setClawReleaseLowerAngle),                            // Release the lower pixel
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let it drop
                new InstantCommand(_claw::setClawCloseAngle),                                   // Close the claw
                new InstantCommand(() -> _arm.setArmData(35,30,0)),                             // Set arm to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate back to center
                new AutoDriveToDistance(_opMode,_drive,-500, 0.4, 0,0,2),                       // Drive back away from the spike marks
                new AutoRotateRobot(_opMode,_drive, -90,0.25,3),                                // Rotate to go under the truss
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.8, -90,2.05),                      // Drive under the truss
                new AutoRotateRobot(_opMode,_drive, 90,0.25,3),                                 // Rotate to the backdrop
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.4, 90,3.05),                       // Drive to the wall
                new InstantCommand(_claw::setClawOpenAngle),                                    // Open the claw
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.4, 90,.6),                          // Backup a little
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the arm drop
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
