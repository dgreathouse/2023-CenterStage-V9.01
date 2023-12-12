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
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingRedWallToBackdrop extends SequentialCommandGroup {

    public GWingRedWallToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        addCommands(
                new WingRedStartCommand(_opMode, _drive, _arm, _claw),
                new AutoRotateRobot(_opMode,_drive, 90,0.25,3),                                 // Rotate to go under truss
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.8, 90,2.05),                        // Drive under truss
                new InstantCommand(() -> _arm.setArmData(42,30,0)),                             // Raise the arm to the backdrop
                new AutoDriveToBackdropFromWing(_opMode,_drive),                                // Drive to the backdrop
                new InstantCommand(_claw::setClawReleaseUpperAngle),                            // Release the upper pixel
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, -90,0.6),                       // Drive back a little
                new AutoDriveToPark(_opMode, _drive, Direction.RIGHT),                          // Park to the right
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay so the arm comes down
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
