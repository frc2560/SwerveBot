package frc.robot.commands.coral;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class GetCoralCommandGroup extends ParallelCommandGroup {
    public GetCoralCommandGroup(Swerve drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        super(new CoralIntakeCommand(coralSubsystem), new SetCoralArmCommand(coralSubsystem));
    }
}