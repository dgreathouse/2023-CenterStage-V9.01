package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdropFromWingMiddle;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class WingMiddleToBackdrop extends SequentialCommandGroup {

    public WingMiddleToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {
        double sign = GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE ? 1.0 : -1;
        addCommands(

                new AutoDriveTimeVel(_opMode, _drive, 90 * sign, 0.8, 0,1.1),                   // Drive to the audience
                new AutoRotateRobot(_opMode,_drive, 90 * sign,0.25,3),                          // Rotate to backdrop
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.7, 90 * sign,1.8),                   // Drive forward to middle of field
                new AutoDriveTimeVel(_opMode, _drive, -90* sign, 0.7, 90* sign,3.05),           // Drive under the truss
                new InstantCommand(() -> _arm.setArmData(35,20,0)),                             // Set arm to backdrop
                new AutoDelayCommand(_opMode, 0.0),                                             // TODO: add a delay if needed to wait for the other team
                new AutoDriveToBackdropFromWingMiddle(_opMode,_drive),                          // Drive to the backdrop
                new InstantCommand(_claw::setClawReleaseUpperAngle),                            // Release the upper pixel
                new AutoDriveTimeVel(_opMode, _drive, 90* sign, 0.5, 90* sign,0.6),             // Drive back a little
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay so the arm comes down
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list


        );

    }
}
