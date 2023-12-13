package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.StartCommand;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.WingMiddleToBackdrop;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingMiddleToBackdrop extends SequentialCommandGroup {

    public GWingMiddleToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        addCommands(
                new StartCommand(_opMode, _drive, _arm, _claw),
                new WingMiddleToBackdrop(_opMode, _drive, _arm, _claw)


        );

    }
}
