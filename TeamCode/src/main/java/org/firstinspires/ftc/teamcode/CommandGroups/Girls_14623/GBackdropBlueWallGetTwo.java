package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToDistance;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropBlueWallGetTwo extends SequentialCommandGroup {

    public GBackdropBlueWallGetTwo(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(
                new InstantCommand(_drive::resetYaw),                                           // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new InstantCommand(() -> _arm.setArmData(35,-10,0)),                            // Raise Arm and lower claw
                new AutoDriveToDistance(_opMode,_drive,620, 0.5, 0,0,3),                        // Drive to team prop
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.CENTER)),   // Check the center
                new AutoRotateRobot(_opMode,_drive, 65,0.3,3),                                  // Rotate to the right
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.LEFT)),     // Check the Right
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.RIGHT)),    // Check the Left
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new InstantCommand(() -> _arm.setArmData(25,-12,0)),                            // Lower arm to drop the pixel
                new AutoDelayCommand(_opMode, .5),                                              // Delay for arm to lower
                new InstantCommand(_claw::setClawReleaseLowerAngle),                            // Drop the lower pixel
                new AutoDelayCommand(_opMode, .5),                                              // Delay so the pixel can drop
                new InstantCommand(_claw::setClawCloseAngle),                                   // Close the claw so it does not come out
                new InstantCommand(() -> _arm.setArmData(35,30,0)),                             // Raise the arm and set claw angle to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate robot back to center
                new AutoDriveToDistance(_opMode,_drive,-500, 0.4, 0,0,2),                       // Drive back away from the spike marks
                new AutoDriveToBackdrop(_opMode,_drive),                                        // Drive to the backdrop
                new InstantCommand(_claw::setClawReleaseUpperAngle),                            // Release the upper pixel
                new AutoDelayCommand(_opMode, .75),                                             // Delay to let the pixel drop
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.5, 90,.75),                         // Back up a little
                new AutoDriveToPark(_opMode, _drive, Direction.LEFT),                           // Goto the right
                new InstantCommand(() -> _arm.setArmData(35,-6,0)),                             // Raise the arm
                new InstantCommand(_claw::setClawOpenAngle),                                    // Open claw
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.7, 90,3.2),                         // Drive under truss to opposite wall
                new AutoRotateRobot(_opMode,_drive, -90,0.25,3),                                // Rotate robot
                new InstantCommand(() -> _arm.setArmData(11,-6,0)),                             // Lower the arm to the stack of 5
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.7, -90,1),                           // Drive to the stack of 5
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.25, -90,.9,0,0),                    // Drive into the stack of 5
                new InstantCommand(_claw::setClawCloseAngle),                                   // Grab the pixels
                new AutoDelayCommand(_opMode, .75),                                             // Delay for claw to close
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.25, -90,.3,0,0),                   // Back up a little
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.7, -90,1.1),                       // Drive back to the wall
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,1,.75,0),                   // Drive back a little to raise the arm
                new InstantCommand(() -> _arm.setArmData(35,-6,0)),                             // Raise the arm
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,1.2,0,0),                   // Drive under truss
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, 90,1.4,0,.5),                   // Drive to backstage and rotate robot
                new InstantCommand(_claw::setClawOpenAngle),                                    // Release the pixels
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the arm lower
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
