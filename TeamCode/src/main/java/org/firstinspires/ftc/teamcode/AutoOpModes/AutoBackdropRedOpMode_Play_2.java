package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroups.Boys_22291.BAutoBackdropRed_Play_2;
import org.firstinspires.ftc.teamcode.CommandGroups.Girls_14623.GAutoBackdropRed_Play_2;
import org.firstinspires.ftc.teamcode.Commands.Arm.AutoArmDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.AutoArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoClawGripSubsystem;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Backdrop Red Play 2", group = "Auto Backdrop Red")
public class AutoBackdropRedOpMode_Play_2 extends CommandOpMode {
    Timing.Timer m_timer;

    Hw hw;
    AutoDriveSubsystem drive;
    AutoDriveDefaultCommand driveDefaultCommand;
    AutoArmDefaultCommand armDefaultCommand;
    AutoArmSubsystem arm;
    AutoClawGripSubsystem claw;

    GAutoBackdropRed_Play_2 Gauto;
    BAutoBackdropRed_Play_2 Bauto;

    @Override
    public void initialize() {
        hw = new Hw(this);
        hw.init();

        // Create Subsystems
        drive = new AutoDriveSubsystem(this);
        arm = new AutoArmSubsystem(this);

        // Create Commands
        driveDefaultCommand = new AutoDriveDefaultCommand(this,drive);
        drive.setDefaultCommand(driveDefaultCommand);
        armDefaultCommand = new AutoArmDefaultCommand(this,arm);
        arm.setDefaultCommand(armDefaultCommand);
        createCommandGroup();
        // Register subsystems
        register(drive,arm,claw);


        m_timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);
        m_timer.start();
        GlobalData.TeamPropLocation = TeamPropLocation.NONE;
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
            // Calculate the run rate of this loop
            telemetry.addData("CPU Load Auto %", 100 - m_timer.remainingTime());
            telemetry.addData("TeamPropLocation", GlobalData.TeamPropLocation);
            telemetry.update();
            // wait till timer is > 50ms to try an create a stable run rate
            // TODO: Decide if timer is needed and helps with accuracy of driving and change accordingly
            if(k.SYSTEM.isLoopRateLimited){while(!m_timer.done()){} m_timer.start();}


        }
        reset();
    }
    private void createCommandGroup() {
        if(GlobalData.TeamNumber == 22291) {
            Bauto = new BAutoBackdropRed_Play_2(this, drive,arm,claw);
        }else {
            Gauto = new GAutoBackdropRed_Play_2(this, drive,arm,claw);
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
