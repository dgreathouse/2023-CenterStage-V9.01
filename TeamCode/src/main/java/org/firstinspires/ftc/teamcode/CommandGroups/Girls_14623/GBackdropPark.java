package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.DriveToBackdrop;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.StartCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GBackdropPark extends SequentialCommandGroup {
    /**
     *
     * @param _opMode The OpMode
     * @param _drive The Drive subsystem reference
     * @param _arm The Arm subsystem reference
     * @param _claw The Claw subsystem reference
     */
    public GBackdropPark(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        addCommands(
                new StartCommand(_opMode, _drive, _arm, _claw),
                new DriveToBackdrop(_opMode, _drive, _arm, _claw),
                new AutoDriveToPark(_opMode, _drive),                           // Park to the left or middle
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the arm lower
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
