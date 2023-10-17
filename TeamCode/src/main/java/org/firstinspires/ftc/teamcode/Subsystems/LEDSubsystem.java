package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;

public class LEDSubsystem extends SubsystemBase {

    RevColorSensorV3 m_chassisStop;

    double green = 1.0;
    double yellow = 0.6;
    double purple = 0.3;
    double white = 0.0;

    CommandOpMode m_opMode;
    public LEDSubsystem(CommandOpMode _opMode){
        m_opMode = _opMode;
        initHardware();

    }
    private void initHardware() {

    }
    @Override
    public void periodic() {

    }

}
