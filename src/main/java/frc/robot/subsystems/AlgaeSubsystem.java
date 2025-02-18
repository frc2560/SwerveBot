package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class AlgaeSubsystem implements Subsystem {

    private SparkMax algaeArm;

    private RelativeEncoder armEncoder;
    private WPI_TalonSRX algaeIntakeLeft;
    private WPI_TalonSRX  algaeIntakeRight;

    private DigitalInput lowerLimitSwitch;
    private DigitalInput hasAlgae;

    public AlgaeSubsystem() {
        algaeArm = new SparkMax(Constants.Algae.ARM_MOTOR, SparkLowLevel.MotorType.kBrushless);
        armEncoder = algaeArm.getEncoder();
        algaeIntakeLeft = new WPI_TalonSRX(Constants.Algae.INTAKE_LEFT);
        algaeIntakeRight = new WPI_TalonSRX(Constants.Algae.INTAKE_RIGHT);
        lowerLimitSwitch = new DigitalInput(Constants.Algae.LOWER_LIMIT_SWITCH);
        hasAlgae = new DigitalInput(Constants.Algae.PhotoSensor);
    }

    public boolean hasAlgae() {
        return hasAlgae.get();
    }

    public void intakeAlgae()
    {
        algaeIntakeLeft.set(Constants.Algae.INTAKE_SPEED);
        algaeIntakeRight.set(Constants.Algae.INTAKE_SPEED);
    }

    public void outtakeAlgae()
    {
        algaeIntakeLeft.set(-Constants.Algae.INTAKE_SPEED);
        algaeIntakeRight.set(-Constants.Algae.INTAKE_SPEED);
    }

    public void stopIntake()
    {
        algaeIntakeLeft.set(0);
        algaeIntakeRight.set(0);
    }

    public double getArmPosition()
    {
        return armEncoder.getPosition();
    }

    public void armUp()
    {
        algaeArm.set(Constants.Algae.ARM_SPEED);
    }

    public void armDown()
    {
        algaeArm.set(-Constants.Algae.ARM_SPEED);
    }

    public void stopArm()
    {
        algaeArm.set(0);
    }

    public boolean isLowerLimitSwitchPressed()
    {
        return lowerLimitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaeArmPosition",getArmPosition());
        if(isLowerLimitSwitchPressed())
        {
            armEncoder.setPosition(0);
        }
    }
}


