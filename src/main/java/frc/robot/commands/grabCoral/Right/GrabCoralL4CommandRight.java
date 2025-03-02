package frc.robot.commands.grabCoral.Right;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.RaiseCoralArmCommand;
import frc.robot.commands.coral.SetCoralArmL1L2L3Command;
import frc.robot.commands.coral.SetCoralArmL4Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL3Command;
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
                        Constants.AlignToTag.RightReef_OMEGA_STAGE).withTimeout(3),
                new GoToL4Command(elevatorSubsystem),
                new SetCoralArmL4Command(coralSubsystem),
                new CoralOutTakeCommand(coralSubsystem).withTimeout(1),
                new RaiseCoralArmCommand(coralSubsystem).alongWith(new GoToL4Command(elevatorSubsystem)),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}



