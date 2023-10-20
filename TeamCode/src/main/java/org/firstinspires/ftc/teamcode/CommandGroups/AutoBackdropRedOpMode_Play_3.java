package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroups.22291.BAutoBackdropRed_Play_3;
import org.firstinspires.ftc.teamcode.CommandGroups.14623.GAutoBackdropRed_Play_3;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Backdrop Red Play 3", group = "Auto Wing Red")
public class AutoBackdropRedOpMode_Play_3 extends CommandOpMode {
    Timing.Timer m_timer;

    double m_timerAvg = 0.0;
    double m_timerCnt = 0;
    Hw hw;
    DriveSubsystem drive;
    ArmSubsystem arm;
    GAutoBackdropRed_Play_3 Gauto;
    BAutoBackdropRed_Play_3 Bauto;

    @Override
    public void initialize() {
        hw = new Hw(this);
        hw.init();

        // Create Subsystems
        drive = new DriveSubsystem(this);
        arm = new ArmSubsystem(this);

        // Create Commands
        createCommandGroup();

        // Register subsystems
        register(drive,arm);

        m_timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);
        m_timer.start();
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
            telemetry.update();
            // wait till timer is > 50ms to try an create a stable run rate
            while(!m_timer.done()){} m_timer.start();

        }
        reset();
    }
    private void createCommandGroup() {
        if(GlobalData.TeamNunber == 22291) {
        	Gauto = new BAutoBackdropRed_Play_3(this, drive,arm);
        }else {
        	Bauto = new GAutoBackdropRed_Play_3(this, drive,arm);
        }

    }
    private scheduleCommandGroup() {
        if(GlobalData.TeamNunber == 22291) {
        	CommandScheduler.getInstance().schedule(Bauto);
        }else {
        	CommandScheduler.getInstance().schedule(Gauto);
        }
    }
}
