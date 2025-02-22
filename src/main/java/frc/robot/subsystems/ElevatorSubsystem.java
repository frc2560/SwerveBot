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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public  class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final DigitalInput elevatorBottomSwitch;
    private final RelativeEncoder elevatorEncoder;

    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);
        elevatorBottomSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_SWITCH);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        elevatorMotor.set(speed);
    }


    public boolean getBottomSwitch() {
        return !elevatorBottomSwitch.get();
    }

    public double getPosition() {
        return elevatorEncoder.getPosition();
    }

    public void stopElevator() {
        elevatorMotor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ElevatorPosition", getPosition());
        SmartDashboard.putBoolean("ElevatorSwitch", getBottomSwitch());
        //TODO figure out Range of Position for elevator
        if (getBottomSwitch()) {
            elevatorEncoder.setPosition(0);
        }
    }
}

