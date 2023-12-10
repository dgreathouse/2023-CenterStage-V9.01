package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropBlueParkMiddle extends SequentialCommandGroup {

    public GBackdropBlueParkMiddle(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(
                new InstantCommand(_drive::resetYaw),                                           // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new InstantCommand(() -> _arm.setArmData(35,-10,0)),                            // Raise Arm and lower claw
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.5, 0,1.82, 0.65, 0.65),                // Drive to team prop
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.CENTER)),   // Check the center
                new AutoRotateRobot(_opMode,_drive, 65,0.25,3),                                // Rotate to the right
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.LEFT)),    // Check the Right
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.RIGHT)),     // Check the Left
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new InstantCommand(() -> _arm.setArmData(25,-12,0)),                            // Lower arm to drop the pixel
                new AutoDelayCommand(_opMode, 1),
                new InstantCommand(_claw::setClawReleaseLowerAngle),                            // Drop the lower pixel
                new AutoDelayCommand(_opMode, 1),                                               // Delay so the pixel can drop
                new InstantCommand(_claw::setClawCloseAngle),                                   // Close the claw so it does not come out
                new InstantCommand(() -> _arm.setArmData(35,30,0)),                             // Raise the arm and set claw angle to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate robot back to center
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.6, 0,1.6),                        // Drive back away from team prop
                new AutoDriveToBackdrop(_opMode,_drive),                                        // Drive to the backdrop
                new InstantCommand(_claw::setClawReleaseUpperAngle),                            // Release the upper pixel
                new AutoDelayCommand(_opMode, .75),                                             // Delay to let the pixel drop
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.5, 90,.75),                       // Back up a little
                new AutoDriveToPark(_opMode, _drive, Direction.RIGHT),                          // Park to the left or middle
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 2),                                               // Delay to let the arm lower
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
