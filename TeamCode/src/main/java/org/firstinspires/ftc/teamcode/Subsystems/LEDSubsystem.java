package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.PixelColor;
import org.firstinspires.ftc.teamcode.Lib.k;

public class LEDSubsystem extends SubsystemBase {


    double green = 1.0;
    double yellow = 0.6;
    double purple = 0.3;
    double white = 0.0;
    ServoEx m_topServo;
    ServoEx m_botServo;
    CommandOpMode m_opMode;
    public LEDSubsystem(CommandOpMode _opMode){
        m_opMode = _opMode;
        initHardware();

    }
    private void initHardware() {
        m_topServo = new SimpleServo(m_opMode.hardwareMap, Hw.s_ledTop, 0,180, AngleUnit.DEGREES);
        m_botServo = new SimpleServo(m_opMode.hardwareMap,Hw.s_ledBot, 0,180, AngleUnit.DEGREES);
    }
    public void setTopServoPosition(PixelColor _color){
        setServoColor(_color, m_topServo);
    }
    public void setBotServoPosition(PixelColor _color){
        setServoColor(_color, m_botServo);
    }
    private void setServoColor(PixelColor _color, ServoEx _servo){
        switch (_color){
            case GREEN:
                _servo.setPosition(k.LEDS.GreenServoValue);
                break;
            case WHITE:
                _servo.setPosition(k.LEDS.WhiteServoValue);
                break;
            case PURPLE:
                _servo.setPosition(k.LEDS.PurpleServoValue);
                break;
            case YELLOW:
                _servo.setPosition(k.LEDS.YellowServoValue);
                break;

            default:
                _servo.setPosition(k.LEDS.OffServoValue);
                break;
        }
    }

    @Override
    public void periodic() {

    }

}
