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
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToPark;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveToTeamProp;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.ArmData;
import org.firstinspires.ftc.teamcode.Lib.ArmPos;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

public class GAutoBackdropRedParkMiddle extends SequentialCommandGroup {

    public GAutoBackdropRedParkMiddle(CommandOpMode _opMode, AutoDriveSubsystem _drive, AutoArmSubsystem _arm, AutoClawGripSubsystem _claw) {

        ArmData armData = new ArmData();
        addCommands(
                new AutoDelayCommand(_opMode, 0.5),                                                         // Wait for claw to close
                new ArmAutoGotoPosition(_opMode,_arm,35.0,-10.0, 0.0),          // Raise the arm
                new AutoDelayCommand(_opMode, 0.25),                                                        // Wait for arm to raise before driving
                new AutoDriveTimeVel(_opMode, _drive, 0, 0.5, 0, 1.2),       // Drive out closer to the Team Prop
                new ArmGetTeamPropLocation(_opMode, _arm, TeamPropLocation.CENTER),
                new AutoRotateRobot(_opMode,_drive, 30.0,0.3, 2.0),
                new ArmGetTeamPropLocation(_opMode,_arm, TeamPropLocation.RIGHT),
                new ArmGetTeamPropLocation(_opMode,_arm, TeamPropLocation.LEFT),

                new AutoStopOpModeCommand(_opMode)                                                                  // Stop the opMode
        );

    }
}
