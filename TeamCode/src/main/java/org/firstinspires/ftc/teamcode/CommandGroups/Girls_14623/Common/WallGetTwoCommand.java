package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToWallFromBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class WallGetTwoCommand extends SequentialCommandGroup {

    public WallGetTwoCommand(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {
        double sign = GlobalData.MATCH.AutoTeamColor == TeamColor.BLUE ? 1.0 : -1;
        addCommands(


                new InstantCommand(() -> _arm.setArmData(35,-6,0)),                             // Raise the arm
                new InstantCommand(_claw::setClawOpenAngle),                                    // Open claw
                new AutoDriveToWallFromBackdrop(_opMode, _drive),
                new AutoDriveTimeVel(_opMode, _drive, 90 * sign, 0.7, 90 * sign,3.2),           // Drive under truss to opposite wall
                new InstantCommand(() -> _arm.setArmData(11,-6,0)),                             // Lower the arm to the stack of 5
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.7, -90* sign,1),                     // Drive to the stack of 5
                new AutoDriveTimeVel(_opMode, _drive, 90* sign, 0.25, -90 * sign,.9,0,0),       // Drive into the stack of 5
                new InstantCommand(_claw::setClawCloseAngle),                                   // Grab the pixels
                new AutoDelayCommand(_opMode, .75),                                             // Delay for claw to close
                new AutoDriveTimeVel(_opMode, _drive, -90 * sign, 0.25, -90 * sign,.3,0,0),     // Back up a little
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.7, -90 * sign,1.1),                // Drive back to the wall
                new AutoDriveTimeVel(_opMode, _drive, -90* sign, 0.7, -90* sign,1,.75,0),       // Drive back a little to raise the arm
                new InstantCommand(() -> _arm.setArmData(35,-6,0)),                             // Raise the arm
                new AutoDriveTimeVel(_opMode, _drive, -90 * sign, 0.7, -90 * sign,1.2,0,0),     // Drive under truss
                new AutoDriveTimeVel(_opMode, _drive, -90 * sign, 0.5, 90 * sign,1.4,0,.5),     // Drive to backstage and rotate robot
                new InstantCommand(_claw::setClawOpenAngle),                                    // Release the pixels
                new InstantCommand(() -> _arm.setArmData(0,0,0)),                               // Lower the arm
                new AutoDelayCommand(_opMode, 1),                                               // Delay to let the arm lower
                new InstantCommand(_opMode::requestOpModeStop)                                  // This must be the last line of every command list
        );

    }
}
