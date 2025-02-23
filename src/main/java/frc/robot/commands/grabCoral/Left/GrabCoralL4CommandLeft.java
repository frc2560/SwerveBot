package frc.robot.commands.grabCoral.Left;

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


public class GrabCoralL4CommandLeft extends SequentialCommandGroup {
    public GrabCoralL4CommandLeft(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        super(
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.LeftReef_AREA_STAGE,
                        Constants.AlignToTag.LeftReef_Y_STAGE,
                        Constants.AlignToTag.LeftReef_OMEGA_STAGE),
                new SetCoralArmL4Command(coralSubsystem),
                new GoToL4Command(elevatorSubsystem),
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.LeftReef_AREA_REEF,
                        Constants.AlignToTag.LeftReef_Y_STAGE,
                        Constants.AlignToTag.LeftReef_OMEGA_STAGE),
                new CoralOutTakeCommand(coralSubsystem),
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.LeftReef_AREA_STAGE,
                        Constants.AlignToTag.LeftReef_Y_STAGE,
                        Constants.AlignToTag.LeftReef_OMEGA_STAGE),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



