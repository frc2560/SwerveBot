package frc.robot.commands.grabCoral.Right;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.SetCoralArmL4Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL4Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;


public class GrabCoralL4CommandRight extends SequentialCommandGroup {
    public GrabCoralL4CommandRight(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        super(
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.RightReef_AREA_STAGE,
                        Constants.AlignToTag.RightReef_Y_STAGE,
                        Constants.AlignToTag.RightReef_OMEGA_STAGE),
                new SetCoralArmL4Command(coralSubsystem),
                new GoToL4Command(elevatorSubsystem),
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.RightReef_AREA_REEF,
                        Constants.AlignToTag.RightReef_Y_STAGE,
                        Constants.AlignToTag.RightReef_OMEGA_STAGE),
                new CoralOutTakeCommand(coralSubsystem),
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.RightReef_AREA_STAGE,
                        Constants.AlignToTag.RightReef_Y_STAGE,
                        Constants.AlignToTag.RightReef_OMEGA_STAGE),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



