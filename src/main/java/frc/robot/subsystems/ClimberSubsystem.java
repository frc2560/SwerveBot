package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax climber;
    public ClimberSubsystem() {

        climber = new SparkMax(Constants.Climber.CLIMBER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsyste//       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }


    public void climberArmUp()
    {
        climber.set(Constants.Climber.ARM_SPEED);
    }

    public void climberArmDown()
    {
        climber.set(-Constants.Climber.ARM_SPEED);
    }

    public void stopArm()
    {
        climber.set(0);
    }
}

