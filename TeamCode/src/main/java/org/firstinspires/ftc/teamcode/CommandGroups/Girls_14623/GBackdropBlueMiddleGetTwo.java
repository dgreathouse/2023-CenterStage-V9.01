package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToMiddleFromBackdrop;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropBlueMiddleGetTwo extends SequentialCommandGroup {

    public GBackdropBlueMiddleGetTwo(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(
                new BackdropBlueStartCommand(_opMode,_drive,_arm,_claw),
                new AutoDriveToMiddleFromBackdrop(_opMode,_drive),                              // Drive to the middle of the field facing the audience
                new InstantCommand(_claw::setClawOpenAngle),                                    // Open the claw to be ready to grab them
                new InstantCommand(() -> _arm.setArmData(35,-6,0)),                             // Raise the arm and set claw angle lower to get a pixel
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.7, -90,2.85,.75,.75),               // Drive under truss
                new InstantCommand(() -> _arm.setArmData(12,-6,0)),                             // Lower arm to grab pixels
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.25, -90,1.80, 0, 0.75),             // Drive to pixel
                new InstantCommand(_claw::setClawCloseAngle),                                   // Grab the pixels
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.7, -90,1, 1, 0),                   // Drive back a little
                new InstantCommand(() -> _arm.setArmData(35,-6,0)),                             // Raise the arm
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.9, -90,1.30, 0, 0),                // Drive under truss
                new InstantCommand(() -> _arm.setArmData(5,-6,0)),                              // Set arm to floor
                new AutoDriveTimeVel(_opMode, _drive, -120, 0.5, 90,0.8, 0, 0),                 // Drive to backstage
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, 90,0.9, 0, 1),                  // Drive to backstage
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm to the floor
                new InstantCommand(_claw::setClawOpenAngle),                                    // Open claw
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the arm lower
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
