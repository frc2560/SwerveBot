package frc.robot.commands.grabCoral;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.coral.CoralIntakeCommand;
import frc.robot.commands.coral.SetCoralArmFeederCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class GetCoralCommandGroup extends ParallelCommandGroup {
    public GetCoralCommandGroup(Swerve drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        super(new CoralIntakeCommand(coralSubsystem), new SetCoralArmFeederCommand(coralSubsystem));
    }
}