package frc.robot.commands.grabCoral;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.CoralIntakeCommand;
import frc.robot.commands.coral.RaiseCoralArmCommand;
import frc.robot.commands.coral.SetCoralArmFeederCommand;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL1Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class GetCoralFeederCommandGroup extends SequentialCommandGroup {
    public GetCoralFeederCommandGroup(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new GoToL1Command(elevatorSubsystem),
                new SetCoralArmFeederCommand(coralSubsystem),
                new CoralIntakeCommand(coralSubsystem),
                new RaiseCoralArmCommand(coralSubsystem),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}