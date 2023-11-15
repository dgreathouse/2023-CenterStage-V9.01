package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CommandGroups.Boys_22291.BAutoBackdropBlue_Play_3;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.GAutoBackdropBlue_Play_3;
import org.firstinspires.ftc.teamcode.Commands.Arm.AutoArmDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.AutoFieldLocation;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.TeamColor;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Backdrop Blue Play 3", group = "Auto Backdrop Blue")
public class AutoBackdropBlueOpMode_Play_3 extends CommandOpMode {
    Timing.Timer m_timer;

    Hw hw;
    AutoDriveSubsystem drive;
    AutoDriveDefaultCommand driveDefaultCommand;
    AutoArmDefaultCommand armDefaultCommand;
    AutoArmSubsystem arm;
    AutoClawGripSubsystem claw;
    GAutoBackdropBlue_Play_3 Gauto;
    BAutoBackdropBlue_Play_3 Bauto;

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
//        armDefaultCommand = new AutoArmDefaultCommand(this,arm);
//        arm.setDefaultCommand(armDefaultCommand);
        createCommandGroup();
        // Register subsystems
        register(drive,arm,claw);

        m_timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);
        m_timer.start();
        GlobalData.MATCH.TeamColor = TeamColor.BLUE;
        GlobalData.MATCH.FieldLocation = AutoFieldLocation.BACKDROP;
    }
    @Override

    public void runOpMode() throws InterruptedException{
        initialize();

        waitForStart();

        // Schedule the auto play to run
        scheduleCommandGroup();
        //schedule(armDefaultCommand);
        //schedule(clawGripDefaultCommand);

        // run the scheduler
        while (!isStopRequested() || opModeIsActive()) {
            run();

            telemetry.update();
            // wait till timer is > 50ms to try an create a stable run rate
            if(k.SYSTEM.isLoopRateLimited){while(!m_timer.done()){} m_timer.start();}

        }
        reset();
    }
    private void createCommandGroup() {
        if(GlobalData.TeamNumber == 22291) {
            Bauto = new BAutoBackdropBlue_Play_3(this, drive,arm,claw);
        }else {
            Gauto = new GAutoBackdropBlue_Play_3(this, drive,arm,claw);
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
