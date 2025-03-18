package frc.robot.commands.scoreCoral.Right;

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


public class RightScoreCoralL3Command extends SequentialCommandGroup {
    public RightScoreCoralL3Command(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        super(
               new TurnToAngleCommand(swervesubsystem).withTimeout(5),
                new ParallelCommandGroup(
                        new AlignWithTag(swervesubsystem, Constants.AlignToTag.RightReef_AREA_L2L3, Constants.AlignToTag.RightReef_Y_STAGE, Constants.AlignToTag.RightReef_OMEGA_STAGE),
                        new GoToL3Command(elevatorSubsystem)
                ).withTimeout(5),
                new SetCoralArmL1L2L3Command(coralSubsystem),
                new CoralOutTakeCommand(coralSubsystem).withTimeout(1),
                new RaiseCoralArmCommand(coralSubsystem),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



