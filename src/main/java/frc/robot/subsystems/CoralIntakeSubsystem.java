package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class CoralIntakeSubsystem implements Subsystem {

    private SparkMax coralIntake;
    private SparkMax croalArm;
    private RelativeEncoder armEncoder;
    private DigitalInput lowerSwitch;
    private DigitalInput hasCoral;

    public CoralIntakeSubsystem() {
       coralIntake = new SparkMax(Constants.Coral.IntakeMotor, SparkLowLevel.MotorType.kBrushless);
       croalArm = new SparkMax(Constants.Coral.ArmMotor, SparkLowLevel.MotorType.kBrushless);
       lowerSwitch = new DigitalInput(Constants.Coral.LowerSwitch);
        hasCoral = new DigitalInput(Constants.Coral.PhotoSensor);
       armEncoder = croalArm.getEncoder();
    }

    public boolean hasCoral() {
        return hasCoral.get();
    }

    public void intakeCoral()
    {
        coralIntake.set(Constants.Coral.IntakeSpeed);
    }

    public void outtakeCoral()
    {
        coralIntake.set(-Constants.Coral.OutTakeSpeed);
    }

    public void moveArmUp()
    {
        croalArm.set(Constants.Coral.ArmSpeed);
    }

    public void moveArmDown()
    {
        croalArm.set(-Constants.Coral.ArmSpeed);
    }

    public void stopIntake()
    {
        coralIntake.set(0);
    }

    public void stopArm()
    {
        croalArm.set(0);
    }

    public double getArmLocation()
    {
        return armEncoder.getPosition();
    }

    public boolean isLowerSwitchPressed()
    {
        return lowerSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CoralArm", getArmLocation());
        if(isLowerSwitchPressed())
        {
            armEncoder.setPosition(0);
        }
    }
}

