package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveSegments;
import org.firstinspires.ftc.teamcode.Lib.Segments.Segments;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class AutoDriveTurnCommandGroup extends SequentialCommandGroup {

    public AutoDriveTurnCommandGroup(CommandOpMode _opMode, DriveSubsystem _drive) {
        addCommands(
                //new AutoDetectAprilTag(_opMode, 30)
            //new AutoDriveDistance(_opMode,_drive,50,.7, DAngle.ang_0,10)
               // new AutoDriveTimeVel(_opMode,_drive,0,0.5,0,2.0),
              //  new AutoDriveTimeVel(_opMode,_drive,90,0.5,90,2.0),
                new AutoDriveSegments(_opMode, _drive,Segments.figure8),
                new AutoStopOpModeCommand(_opMode)

        );

    }
}
