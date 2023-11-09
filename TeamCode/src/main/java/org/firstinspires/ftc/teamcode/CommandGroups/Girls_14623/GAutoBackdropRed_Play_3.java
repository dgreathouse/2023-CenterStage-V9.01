package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmAutoGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoDetectAprilTag;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdropAprilTag;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;

public class GAutoBackdropRed_Play_3 extends SequentialCommandGroup {

    public GAutoBackdropRed_Play_3(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {
        GlobalData.TeamColor = TeamColor.RED;
        GlobalData.FieldLocation = AutoFieldLocation.BACKDROP;
        addCommands(
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),
            //    new ArmAutoGotoPosition(_opMode, _arm, 40,6,0),
            //    new AutoDriveTimeVel(_opMode, _drive,55,0.4,-90,2200),
                new AutoDetectAprilTag(_opMode,20),
             //   new AutoDriveToBackdropAprilTag(_opMode,_drive),
                new AutoDelayCommand(_opMode,1000),
                new AutoStopOpModeCommand(_opMode) // This must be the last line of every command list
                // new AutoDelayCommand(_opMode,1000),

        );

    }
}
