package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralSubsystem extends SubsystemBase {

    private SparkMax coralIntake;
    private SparkMax coralArm;
    private RelativeEncoder armEncoder;
    private DigitalInput upperSwitch;
    private DigitalInput hasCoral;

    public CoralSubsystem() {
       coralIntake = new SparkMax(Constants.Coral.IntakeMotor, SparkLowLevel.MotorType.kBrushless);
       coralArm = new SparkMax(Constants.Coral.ArmMotor, SparkLowLevel.MotorType.kBrushless);
       upperSwitch = new DigitalInput(Constants.Coral.C_UPPER_LIMIT);
        hasCoral = new DigitalInput(Constants.Coral.PhotoSensor);
       armEncoder = coralArm.getEncoder();
    }

    public boolean hasCoral() {
        return !hasCoral.get();
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
        coralArm.set(Constants.Coral.ArmSpeed);
    }

    public void moveArmDown()
    {
        coralArm.set(-Constants.Coral.ArmSpeed);
    }

    public void stopIntake()
    {
        coralIntake.set(0);
    }

    public void stopArm()
    {
        coralArm.set(0);
    }

    public double getArmLocation()
    {
        return -armEncoder.getPosition();
    }

    public boolean isUpperSwitchPressed()
    {
        return !upperSwitch.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("CoralArmPosition", getArmLocation());
        SmartDashboard.putBoolean("UpperCoralSwitch", isUpperSwitchPressed());
        SmartDashboard.putBoolean("HasCoral", hasCoral());
        if(isUpperSwitchPressed())
        {
            armEncoder.setPosition(0);
        }
    }
}

