package frc.robot.commands.grabCoral.Right;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.SetCoralArmL1L2L3Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL2Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;


public class GrabCoralL2CommandRight extends SequentialCommandGroup {
    public GrabCoralL2CommandRight(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        super(
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.RightReef_AREA_STAGE,
                        Constants.AlignToTag.RightReef_Y,
                        Constants.AlignToTag.RightReef_OMEGA),
                new SetCoralArmL1L2L3Command(coralSubsystem),
                new GoToL2Command(elevatorSubsystem),
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.RightReef_AREA_STAGE,
                        Constants.AlignToTag.RightReef_Y,
                        Constants.AlignToTag.RightReef_OMEGA),
                new CoralOutTakeCommand(coralSubsystem),
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.RightReef_AREA_STAGE,
                        Constants.AlignToTag.RightReef_Y,
                        Constants.AlignToTag.RightReef_OMEGA),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



