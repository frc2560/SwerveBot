package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.algae.IntakeCommand;
import frc.robot.commands.algae.LowerArmCommand;
import frc.robot.commands.elevator.GoToL1AlgaeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class GrabAlgaeL1Command extends SequentialCommandGroup {
    public GrabAlgaeL1Command(ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(  new LowerArmCommand(algaeSubsystem),
                new GoToL1AlgaeCommand(elevatorSubsystem),
                new IntakeCommand(algaeSubsystem));
    }
}