package frc.robot.commands.grabCoral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.SetCoralArmL1L2L3Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL1Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;


public class GrabCoralL1CommandRight extends SequentialCommandGroup {
    public GrabCoralL1CommandRight(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        super(
                new AlignWithTag(
                        swervesubsystem,
                        Constants.AlignToTag.RightReef_AREA_STAGE,
                        Constants.AlignToTag.RightReef_Y,
                        Constants.AlignToTag.RightReef_OMEGA),
                new SetCoralArmL1L2L3Command(coralSubsystem),
                new GoToL1Command(elevatorSubsystem),
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



