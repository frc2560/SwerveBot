package frc.robot.commands.grabCoral.Left;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.RaiseCoralArmCommand;
import frc.robot.commands.coral.SetCoralArmL4Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL4Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.commands.movement.TurnToAngleCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;


public class LeftScoreCoralL4Command extends SequentialCommandGroup {
    public LeftScoreCoralL4Command(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        super(
                //new TurnToAngleCommand(swervesubsystem).withTimeout(0.5),
                new ParallelCommandGroup(
                new AlignWithTag(swervesubsystem, Constants.AlignToTag.LeftReef_AREA_L4, Constants.AlignToTag.LeftReef_Y_STAGE, Constants.AlignToTag.LeftReef_OMEGA_STAGE),
                        new GoToL4Command(elevatorSubsystem)
                ).withTimeout(1.5),
                new GoToL4Command(elevatorSubsystem),
                new SetCoralArmL4Command(coralSubsystem),
                new CoralOutTakeCommand(coralSubsystem).withTimeout(0.75),
                new RaiseCoralArmCommand(coralSubsystem),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



