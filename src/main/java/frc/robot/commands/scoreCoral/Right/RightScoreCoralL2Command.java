package frc.robot.commands.scoreCoral.Right;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.RaiseCoralArmCommand;
import frc.robot.commands.coral.SetCoralArmL1L2L3Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL2Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.commands.movement.TurnToAngleCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;


public class RightScoreCoralL2Command extends SequentialCommandGroup {
    public RightScoreCoralL2Command(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem, boolean isAuto) {
        super(
                new TurnToAngleCommand(swervesubsystem, isAuto),
                new ParallelCommandGroup(
                        new AlignWithTag(swervesubsystem, Constants.AlignToTag.RightReef_AREA_L2, Constants.AlignToTag.RightReef_Y_STAGE, Constants.AlignToTag.RightReef_OMEGA_STAGE),
                        new GoToL2Command(elevatorSubsystem)
                ).withTimeout(2),
                new SetCoralArmL1L2L3Command(coralSubsystem),
                new CoralOutTakeCommand(coralSubsystem).withTimeout(1),
                new RaiseCoralArmCommand(coralSubsystem),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



