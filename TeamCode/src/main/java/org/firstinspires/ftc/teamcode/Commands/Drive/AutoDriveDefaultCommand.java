package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class AutoDriveDefaultCommand extends CommandBase {
    // Declare a variable called "drive" of type "DriveSubsystem"
    DriveSubsystem m_drive;

    // Declare a variable called "opMode" of type "CommandOpMode"
    CommandOpMode m_opMode;
    // Create local variables of type double to store the stick X,Y,Z values and Angle of robot.

    /** Constructor of class
     *
     * @param _opMode The opMode used which will be teleOp or Autonomous
     * @param _drive The DriveSubsystem instance variable
     */
    public AutoDriveDefaultCommand(CommandOpMode _opMode, DriveSubsystem _drive) {

        m_drive = _drive;    // Set the local "m_drive" variable to the parameter "_drive"


        m_opMode = _opMode;  // Set the local "opMode" variable to the parameter "_opMode"
        // Set the requirements for the Command. This always must be done for a "Command"
        // The requirement is any subsystem that will be used by this command.
        addRequirements(m_drive);
    }

    @Override
    public void initialize(){
        m_drive.setIsFieldOriented(true);
        m_drive.disableMotors();
    }

    @Override
    public void execute(){


    }
    @Override
    public void end(boolean _interrupted){

    }

}
