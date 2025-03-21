package frc.robot.commands.scoreCoral;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.RaiseCoralArmCommand;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DownFromFeederCommandGroup extends SequentialCommandGroup {
    public DownFromFeederCommandGroup(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new RaiseCoralArmCommand(coralSubsystem),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}