package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common.StartCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingWallPark extends SequentialCommandGroup {

    public GWingWallPark(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {
        double sign = GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE ? 1.0 : -1;
        addCommands(
                new StartCommand(_opMode, _drive, _arm, _claw),
                new AutoRotateRobot(_opMode,_drive, -90 * sign,0.25,3),                                // Rotate to go under the truss
                new AutoDriveTimeVel(_opMode, _drive, -90 * sign, 0.8, -90 * sign,2.05),                      // Drive under the truss
                new AutoRotateRobot(_opMode,_drive, 90 * sign,0.25,3),                                 // Rotate to the backdrop
                new AutoDriveTimeVel(_opMode, _drive, -90 * sign, 0.4, 90 * sign,3.05),                       // Drive to the wall
                new InstantCommand(_claw::setClawOpenAngle),                                    // Open the claw
                new AutoDriveTimeVel(_opMode, _drive, 90 * sign, 0.4, 90 * sign,.6),                          // Backup a little
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the arm drop
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
