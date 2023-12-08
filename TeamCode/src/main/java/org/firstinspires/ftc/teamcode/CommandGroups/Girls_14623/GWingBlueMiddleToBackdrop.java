package org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Direction;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmAutoGotoPosition;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmGetTeamPropLocation;
import org.firstinspires.ftc.teamcode.Commands.AutoDelayCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoStopOpModeCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawGrip.ClawRotateFingers;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdropFromWingMiddle;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoResetGyroCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateToTeamProp;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GWingBlueMiddleToBackdrop extends SequentialCommandGroup {

    public GWingBlueMiddleToBackdrop(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw)  {

        ArmData armData = new ArmData();
        addCommands(
                new AutoResetGyroCommand(_opMode, _drive),                                              // Reset the gyro
                new AutoDelayCommand(_opMode, 1.0),                                            // Wait for claw to close
                new AutoResetGyroCommand(_opMode, _drive),                                              // Reset the gyro
                new AutoDelayCommand(_opMode, .75),                                            // Wait for claw to close
                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.6, 0,1),
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),
                new AutoRotateRobot(_opMode,_drive, 55,0.25,3),
                new AutoDelayCommand(_opMode, .75),
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.LEFT),
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.RIGHT),
                new AutoRotateToTeamProp(_opMode,_drive),
                new ArmAutoGotoPosition(_opMode, _arm, 20, -12, 0),
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseLowerAngle()),
                new AutoDelayCommand(_opMode, 1),
                new ClawRotateFingers(_opMode, _claw, _claw.getClawCloseAngle()),
                new ArmAutoGotoPosition(_opMode, _arm, 35, -10, 0),
                new AutoRotateRobot(_opMode,_drive, 0,0.25,3),
                new AutoDriveTimeVel(_opMode, _drive, 180, 0.6, 0,1),
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.6, 0,1.5),
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.6, 0,2.5),
                new AutoRotateRobot(_opMode,_drive, 90,0.25,3),
                new AutoDriveTimeVel(_opMode, _drive, 90, 0.8, 90,2.05),
                new ArmAutoGotoPosition(_opMode, _arm, 42, 30, 0),
                new AutoDriveToBackdropFromWingMiddle(_opMode,_drive),
                new ClawRotateFingers(_opMode, _claw, _claw.getClawReleaseUpperAngle()),
                new AutoDriveTimeVel(_opMode, _drive, -90, 0.5, -90,0.6),
                new AutoDriveToPark(_opMode, _drive, Direction.LEFT),
                new ArmAutoGotoPosition(_opMode, _arm, 0, 0, 0),
                new AutoDelayCommand(_opMode, 1),


                new AutoStopOpModeCommand(_opMode)                                                      // This must be the last line of every command list


        );

    }
}
