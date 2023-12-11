package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToDistance;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToMiddleFromBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropBlueMiddleGetTwo extends SequentialCommandGroup {

    public GBackdropBlueMiddleGetTwo(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(
                new InstantCommand(_drive::resetYaw),                                           // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new InstantCommand(() -> _arm.setArmData(35,-10,0)),                            // Raise Arm and lower claw
                new AutoDriveToDistance(_opMode,_drive,620, 0.5, 0,0,3),                        // Drive to team prop
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.CENTER)),   // Check the center
                new AutoRotateRobot(_opMode,_drive, 65,0.25,3),                                 // Rotate to the right
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.LEFT)),     // Check the Right
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.RIGHT)),    // Check the Left
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new InstantCommand(() -> _arm.setArmData(25,-12,0)),                            // Lower arm to drop the pixel
                new AutoDelayCommand(_opMode, 1),                                               // Delay for arm to lower
                new InstantCommand(_claw::setClawReleaseLowerAngle),                            // Drop the lower pixel
                new AutoDelayCommand(_opMode, 1),                                               // Delay so the pixel can drop
                new InstantCommand(_claw::setClawCloseAngle),                                   // Close the claw so it does not come out
                new InstantCommand(() -> _arm.setArmData(35,30,0)),                             // Raise the arm and set claw angle to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate robot back to center
                new AutoDriveToDistance(_opMode,_drive,-500, 0.4, 0,0,2),                       // Drive back away from the spike marks
                new AutoDriveToBackdrop(_opMode,_drive),                                        // Drive to the backdrop
                new InstantCommand(_claw::setClawReleaseUpperAngle),                            // Release the upper pixel
                new AutoDelayCommand(_opMode, .75),                                             // Delay to let the pixel drop
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.5, 90,.75),                         // Back up a little
                new AutoDriveToMiddleFromBackdrop(_opMode,_drive),                              // Drive to the middle of the field facing the audience
                new InstantCommand(_claw::setClawOpenAngle),                                    // Open the claw to be ready to grab them
                new InstantCommand(() -> _arm.setArmData(35,-6,0)),                             // Raise the arm and set claw angle lower to get a pixel
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.7, -90,2.75,.75,.75),               // Drive under truss
                new InstantCommand(() -> _arm.setArmData(12,-6,0)),                             // Lower arm to grab pixels
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.25, -90,1.60, 0, 0.75),             // Drive to pixel
                new InstantCommand(_claw::setClawCloseAngle),                                   // Grab the pixels
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,1, 1, 0),                   // Drive back a little
                new InstantCommand(() -> _arm.setArmData(35,-6,0)),                             // Raise the arm
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,1.50, 0, 0),                // Drive under truss
                new InstantCommand(() -> _arm.setArmData(5,-6,0)),                              // Set arm to floor
                new AutoDriveTimeVel(_opMode, _drive, -120, 0.5, 90,0.8, 0, 0),                 // Drive to backstage
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, 90,0.9, 0, 1),                  // Drive to backstage
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm to the floor
                new InstantCommand(_claw::setClawOpenAngle),                                    // Open claw
                new AutoDelayCommand(_opMode, 2),                                               // Delay to let the arm lower
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
