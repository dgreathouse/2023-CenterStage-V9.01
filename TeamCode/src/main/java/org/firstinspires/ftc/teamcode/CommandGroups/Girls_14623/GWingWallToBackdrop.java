package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.StartCommand;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.WingWallToBackdrop;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingWallToBackdrop extends SequentialCommandGroup {
    /**
     *
     * @param _opMode The OpMode
     * @param _drive The Drive subsystem reference
     * @param _arm The Arm subsystem reference
     * @param _claw The Claw subsystem reference
     */
    public GWingWallToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        addCommands(
                new StartCommand(_opMode, _drive, _arm, _claw),
                new WingWallToBackdrop(_opMode, _drive, _arm, _claw)
        );

    }
}
