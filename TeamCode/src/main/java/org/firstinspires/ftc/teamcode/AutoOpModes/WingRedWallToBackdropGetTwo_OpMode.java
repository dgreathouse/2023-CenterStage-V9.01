package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandGroups.Boys_22291.BWingRedWallToBackdrop;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.GWingWallGetTwo;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.GWingWallToBackdrop;
import org.firstinspires.ftc.teamcode.Commands.Arm.AutoArmDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation_enum;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.ParkDirection;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

@Autonomous(name = "Wing Red Wall To Backdrop Get 2", group = "Wing Red")
@Disabled
public class WingRedWallToBackdropGetTwo_OpMode extends CommandOpMode {
    Timing.Timer m_timer;

    Hw hw;
    AutoDriveSubsystem drive;
    AutoDriveDefaultCommand driveDefaultCommand;
    AutoArmDefaultCommand armDefaultCommand;
    AutoArmSubsystem arm;
    AutoClawGripSubsystem claw;
    GWingWallGetTwo Gauto;
    BWingRedWallToBackdrop Bauto;

    @Override
    public void initialize() {
        hw = new Hw(this);
        hw.init();

        // Create Subsystems
        drive = new AutoDriveSubsystem(this,hw);
        arm = new AutoArmSubsystem(this);
        claw = new AutoClawGripSubsystem(this);

        // Create Commands
        driveDefaultCommand = new AutoDriveDefaultCommand(this,drive);
        drive.setDefaultCommand(driveDefaultCommand);
        armDefaultCommand = new AutoArmDefaultCommand(this,arm);
        arm.setDefaultCommand(armDefaultCommand);
        GlobalData.MATCH.AutoTeamColor = TeamColor.RED;
        GlobalData.MATCH.AutoFieldLocation = AutoFieldLocation_enum.WING;
        GlobalData.MATCH.AutoParkDirection = ParkDirection.WALL;
        createCommandGroup();
        // Register subsystems
        register(drive,arm,claw);


    }
    @Override
    public void runOpMode() throws InterruptedException{
        initialize();

        waitForStart();
        // Schedule the auto play to run
        scheduleCommandGroup();


        // run the scheduler
        while (!isStopRequested() || opModeIsActive()) {
            run();

            telemetry.update();

        }
        reset();
    }
    private void createCommandGroup() {
        if(GlobalData.TeamNumber == 22291) {
            Bauto = new BWingRedWallToBackdrop(this, drive,arm,claw);
        }else {
            Gauto = new GWingWallGetTwo(this, drive,arm,claw);
        }

    }
    private void scheduleCommandGroup() {
        if(GlobalData.TeamNumber == 22291) {
            CommandScheduler.getInstance().schedule(Bauto);
        }else {
            CommandScheduler.getInstance().schedule(Gauto);
        }
    }
}
