package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public  class ElevatorSubsystem implements Subsystem
{
    private final SparkMax elevatorMotor;
    private final SparkMax wristMotor;
    private final DigitalInput elevatorSwitch;
    private final DigitalInput wristSwitch;


    public ElevatorSubsystem()
    {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.



    }
}

