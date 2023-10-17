package org.firstinspires.ftc.teamcode.Commands.LED;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.PixelColor;
import org.firstinspires.ftc.teamcode.Lib.PixelLocation;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.LEDSubsystem;

public class LEDDefaultCommand  extends CommandBase {
    CommandOpMode m_opMode;
    LEDSubsystem m_led;
    PixelColor m_botColor = PixelColor.WHITE;
    PixelColor m_topColor = PixelColor.WHITE;

    ServoEx m_topServo;
    ServoEx m_botServo;
    public LEDDefaultCommand(CommandOpMode _opMode, LEDSubsystem _led){
        m_opMode = _opMode;
        m_led = _led;

    }
    @Override
    public void initialize(){
        m_topServo = new SimpleServo(m_opMode.hardwareMap,Hw.s_ledTop, 0,180, AngleUnit.DEGREES);
        m_botServo = new SimpleServo(m_opMode.hardwareMap,Hw.s_ledBot, 0,180, AngleUnit.DEGREES);
    }
    @Override
    public void execute(){
        // Read the buttons
        setColor();
        setServoColor(m_topColor, m_topServo);
        setServoColor(m_botColor, m_botServo);
        // Set the Servo out to color values
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
    private void setColor(){
        if(Hw.s_gpOperator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (Hw.s_gpOperator.isDown(GamepadKeys.Button.DPAD_UP)) {
                m_topColor = PixelColor.WHITE;
            } else if (Hw.s_gpOperator.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                m_topColor = PixelColor.GREEN;
            } else if (Hw.s_gpOperator.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                m_topColor = PixelColor.YELLOW;
            } else if (Hw.s_gpOperator.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                m_topColor = PixelColor.PURPLE;
            }
        }else {
            if (Hw.s_gpOperator.isDown(GamepadKeys.Button.DPAD_UP)) {
                m_botColor = PixelColor.WHITE;
            } else if (Hw.s_gpOperator.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                m_botColor = PixelColor.GREEN;
            } else if (Hw.s_gpOperator.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                m_botColor = PixelColor.YELLOW;
            } else if (Hw.s_gpOperator.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                m_botColor = PixelColor.PURPLE;
            }
        }

    }
    @Override
    public void end(boolean _interrupted){

    }
}
