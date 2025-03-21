package frc.robot.commands.scoreCoral.Right;

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


public class RightScoreCoralL4Command extends SequentialCommandGroup {
    public RightScoreCoralL4Command(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem, boolean isAuto) {
        super(
                new TurnToAngleCommand(swervesubsystem, isAuto),
                new ParallelCommandGroup(
                        new AlignWithTag(swervesubsystem, Constants.AlignToTag.RightReef_AREA_L4 , Constants.AlignToTag.RightReef_Y_STAGE, Constants.AlignToTag.RightReef_OMEGA_STAGE)
                ).withTimeout(3),
                new TurnToAngleCommand(swervesubsystem, isAuto),
                new GoToL4Command(elevatorSubsystem),
                new SetCoralArmL4Command(coralSubsystem),
                new CoralOutTakeCommand(coralSubsystem).withTimeout(0.75),
                new RaiseCoralArmCommand(coralSubsystem),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



