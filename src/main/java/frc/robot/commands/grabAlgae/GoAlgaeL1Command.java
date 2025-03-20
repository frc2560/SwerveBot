package frc.robot.commands.grabAlgae;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.algae.SetAlgaeArmLowerCommand;
import frc.robot.commands.algae.SetAlgaeArmUpperCommand;
import frc.robot.commands.elevator.GoToL1AlgaeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class GoAlgaeL1Command extends SequentialCommandGroup {
    public GoAlgaeL1Command(ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem) {
        super(

                        new GoToL1AlgaeCommand(elevatorSubsystem),
                        new SetAlgaeArmLowerCommand(algaeSubsystem)
        );
    }
}