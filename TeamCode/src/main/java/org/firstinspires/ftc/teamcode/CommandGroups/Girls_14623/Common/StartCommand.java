package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.Common;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToDistance;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation_enum;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

import java.util.function.BooleanSupplier;

public class StartCommand extends SequentialCommandGroup {
    public StartCommand(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {
        double sign = GlobalData.MATCH.getRotateSign();


        addCommands(
                new InstantCommand(_drive::resetYaw),                                           // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                             // Wait for claw to close
                new InstantCommand(() -> _arm.setArmData(35,-10,0)),                            // Raise Arm and lower claw
                new AutoDriveToDistance(_opMode,_drive,620, 0.5, 0,0,3),                        // Drive to team prop
                new InstantCommand(()-> _arm.checkTeamPropLocation2(TeamPropLocation.CENTER)),  // Check the center
                new AutoRotateRobot(_opMode,_drive, -65,0.25,3),                          // Rotate to the one away from truss
                new InstantCommand(()-> _arm.checkTeamPropLocation2(TeamPropLocation.FIRST)),   // Check the one away from the truss
                new AutoRotateToTeamProp(_opMode,_drive),                                       // Rotate to the team prop
                new InstantCommand(() -> _arm.setArmData(25,-12,0)),                            // Lower arm to drop the pixel
                new AutoDelayCommand(_opMode, 0.5),                                            // Delay for arm to lower
                new InstantCommand(_claw::setClawReleaseLowerAngle),                            // Drop the lower pixel
                new AutoDelayCommand(_opMode, 0.5),                                            // Delay so the pixel can drop
                new InstantCommand(_claw::setClawCloseAngle),                                   // Close the claw so it does not come out
                new InstantCommand(() -> _arm.setArmData(35,30,0)),                             // Raise the arm and set claw angle to backdrop
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),                                  // Rotate robot back to center
                new AutoDriveToDistance(_opMode,_drive,-500, 0.5, 0,0,2)                        // Drive back away from the spike marks

        );
    }
}
