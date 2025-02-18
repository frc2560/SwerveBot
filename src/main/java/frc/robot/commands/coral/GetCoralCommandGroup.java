package frc.robot.commands.coral;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class GetCoralCommandGroup extends ParallelCommandGroup {
    public GetCoralCommandGroup(Swerve drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, CoralIntakeSubsystem coralIntakeSubsystem) {
        super(new CoralIntakeCommand(drivetrainSubsystem, elevatorSubsystem, coralIntakeSubsystem), new SetCoralArmCommand(coralIntakeSubsystem, elevatorSubsystem));
    }
}