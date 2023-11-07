package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.KiwiDrive;

public class DriveSubsystem extends SubsystemBase {
    Hw m_hw;
    // Declare the MotorEx and Vector2D classes for each motor
    private MotorEx m_lDrive, m_rDrive, m_bDrive;
    private KiwiDrive m_drive;
    double drivePIDAngle = 361;

    // Declare a CommandOpMode variable
    private CommandOpMode m_opMode;

    /** Class Constructor
     *
     * @param _opMode The opMode used which will be teleOp or Autonomous
     */
    public DriveSubsystem(CommandOpMode _opMode, Hw _hw) {
        m_opMode = _opMode;
        m_hw = _hw;
        m_hw.initDriveHardware(_opMode,m_lDrive,m_rDrive,m_bDrive);

        m_drive = new KiwiDrive(m_lDrive,m_rDrive, m_bDrive);

    }
    /**
     *
     * @param _strafeSpeed The forward speed in +/- 1 left is positive
     * @param _forwardSpeed The strafe speed in +/- 1 forward is positive
     * @param _zRotation The rotation speed in +/- 1 CCW/left is positive
     */
    public void driveXY(double _strafeSpeed, double _forwardSpeed, double _zRotation) {
        m_drive.driveXY(_strafeSpeed,_forwardSpeed,_zRotation,getRobotAngle());
    }

    public void drivePolar(double _angle, double _speed, double _rot){
        m_drive.drivePolar(_angle,_speed,_rot,getRobotAngle());
    }
    public void toggleIsFieldOriented(){
        m_drive.toggleIsFieldOriented();
    }
    /**
     *
     * @return The robot angle in degrees with CCW as positive
     */
    public double getRobotAngle(){
        YawPitchRollAngles angles = Hw.s_imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
    public void setDrivePIDAngle(double _angle){
        drivePIDAngle = _angle;
    }
    public double getDrivePIDAngle(){
        return drivePIDAngle;
    }
    /**
     * Reset the gyro Yaw which is what is used for the robot angle
     */
    public void resetGyro(){
        Hw.s_imu.resetYaw();
    }

    /**
     * Reset the motor encoders back to zero
     */
    public void resetMotors(){
        m_lDrive.resetEncoder();
        m_bDrive.resetEncoder();
        m_rDrive.resetEncoder();

    }

    public void disableMotors(){
        m_lDrive.set(0);
        m_rDrive.set(0);
        m_bDrive.set(0);
    }
    public void setIsFieldOriented(boolean _val){
        m_drive.setIsFieldOriented(_val);
    }
    @Override
    public void periodic(){

        m_opMode.telemetry.addData("Robot Angle", "%3.3f",getRobotAngle());
        m_opMode.telemetry.addData("Drive Angle", "%3.3f", m_drive.getDriveAngle());
        m_opMode.telemetry.addLine("------------------------------------");
        m_opMode.telemetry.addData("Strafe", "%3.3f", m_drive.getStrafe());
        m_opMode.telemetry.addData("Forward", "%3.3f",m_drive.getForward());
        m_opMode.telemetry.addData("FieldOriented", m_drive.getIsFieldOriented());
        m_opMode.telemetry.addLine("------------------------------------");
    }
}
