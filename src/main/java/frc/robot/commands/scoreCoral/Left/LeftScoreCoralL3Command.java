package frc.robot.commands.scoreCoral.Left;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.RaiseCoralArmCommand;
import frc.robot.commands.coral.SetCoralArmL1L2L3Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL3Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.commands.movement.TurnToAngleCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;


public class LeftScoreCoralL3Command extends SequentialCommandGroup {
    public LeftScoreCoralL3Command(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem, boolean isAuto) {
        super(
                new TurnToAngleCommand(swervesubsystem, isAuto).withTimeout(5),
                new ParallelCommandGroup(
                        new AlignWithTag(swervesubsystem, Constants.AlignToTag.LeftReef_AREA_L3, Constants.AlignToTag.LeftReef_Y_STAGE, Constants.AlignToTag.LeftReef_OMEGA_STAGE),
                        new GoToL3Command(elevatorSubsystem)
                ).withTimeout(5),
                new SetCoralArmL1L2L3Command(coralSubsystem),
                new CoralOutTakeCommand(coralSubsystem).withTimeout(1),
                new RaiseCoralArmCommand(coralSubsystem),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



