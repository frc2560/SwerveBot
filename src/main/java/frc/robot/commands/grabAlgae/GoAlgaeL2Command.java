package frc.robot.commands.grabAlgae;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.algae.SetAlgaeArmLowerCommand;
import frc.robot.commands.algae.SetAlgaeArmUpperCommand;
import frc.robot.commands.elevator.GoToL2AlgaeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class GoAlgaeL2Command extends SequentialCommandGroup {
    public GoAlgaeL2Command(ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem) {
        super(
                        new GoToL2AlgaeCommand(elevatorSubsystem),
                        new SetAlgaeArmLowerCommand(algaeSubsystem)
        );
    }
}