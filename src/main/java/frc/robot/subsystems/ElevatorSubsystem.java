package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public  class ElevatorSubsystem implements Subsystem {
    private final SparkMax elevatorMotor;
    private final DigitalInput elevatorTopSwitch;
    private final DigitalInput elevatorBottomSwitch;
    private final RelativeEncoder elevatorEncoder;

    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);
        elevatorTopSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_TOP_SWITCH);
        elevatorBottomSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_SWITCH);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    public boolean getTopSwitch() {
        return !elevatorTopSwitch.get();
    }

    public boolean getBottomSwitch() {
        return !elevatorBottomSwitch.get();
    }

    public double getPosition() {
        return -(elevatorEncoder.getPosition() / 2);
    }

    public void stopElevator() {
        elevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putBoolean("Bottom Switch", getBottomSwitch());
        SmartDashboard.putBoolean("Top Switch", getTopSwitch());

        if (getTopSwitch()) {
            elevatorEncoder.setPosition(100);
        }

        if (getBottomSwitch()) {
            elevatorEncoder.setPosition(0);

        }
    }
}

