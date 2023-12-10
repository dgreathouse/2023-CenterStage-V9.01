package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdropFromWing;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingRedWallToBackdrop extends SequentialCommandGroup {

    public GWingRedWallToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        addCommands(
                new InstantCommand(_drive::resetYaw),                                      // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new InstantCommand(() -> _arm.setArmData(35,-10,0)),                             // Raise the arm
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.6, 0,1.75),                          // Drive to team prop
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.CENTER)),             // Check the center
                new AutoRotateRobot(_opMode,_drive, 55,0.25,3),                                 // Rotate to the left
                new AutoDelayCommand(_opMode, .75),
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.LEFT)),               // Check the left
                new InstantCommand(()-> _arm.checkTeamPropLocation(TeamPropLocation.RIGHT)),              // Check the right
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new InstantCommand(() -> _arm.setArmData(20,-12,0)),                             // Lower the arm
                new InstantCommand(_claw::setClawReleaseLowerAngle),        // Release the lower pixel
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the pixel fall
                new InstantCommand(_claw::setClawCloseAngle),               // Close the claw
                new InstantCommand(() -> _arm.setArmData(35,-10,0)),                             // Set arm to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate to the center
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.6, 0,1.65),                        // Drive back to the wall
                new AutoRotateRobot(_opMode,_drive, 90,0.25,3),                                 // Rotate to go under truss
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.8, 90,2.05),                        // Drive under truss
                new InstantCommand(() -> _arm.setArmData(42,30,0)),                              // Raise the arm to the backdrop
                new AutoDriveToBackdropFromWing(_opMode,_drive),                                // Drive to the backdrop
                new InstantCommand(_claw::setClawReleaseUpperAngle),        // Release the upper pixel
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, -90,0.6),                       // Drive back a little
                new AutoDriveToPark(_opMode, _drive, Direction.RIGHT),                          // Park to the right
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay so the arm comes down
                new InstantCommand(_opMode::requestOpModeStop)                                              // This must be the last line of every command list
        );

    }
}
