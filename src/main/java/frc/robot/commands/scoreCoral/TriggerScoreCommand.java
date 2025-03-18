package frc.robot.commands.scoreCoral;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.RaiseCoralArmCommand;
import frc.robot.commands.coral.SetCoralArmL4Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class TriggerScoreCommand extends SequentialCommandGroup {
    public TriggerScoreCommand(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new SetCoralArmL4Command(coralSubsystem),
                new CoralOutTakeCommand(coralSubsystem).withTimeout(0.75),
                new RaiseCoralArmCommand(coralSubsystem),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}