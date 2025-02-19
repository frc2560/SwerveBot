package frc.robot.commands.grabCoral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.CoralIntakeCommand;
import frc.robot.commands.coral.CoralOutTakeCommand;
import frc.robot.commands.coral.SetCoralArmL1L2L3Command;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL1Command;
import frc.robot.commands.elevator.GoToL3Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;


public class GrabCoralL3Command extends SequentialCommandGroup {
    public GrabCoralL3Command(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        super(new AlignWithTag(swervesubsystem, Constants.AlignToTag.CenterReef_AREA_REEF, Constants.AlignToTag.CenterReef_Y, Constants.AlignToTag.CenterReef_OMEGA),
                new SetCoralArmL1L2L3Command(coralSubsystem),
                new GoToL3Command(elevatorSubsystem),
                new AlignWithTag(swervesubsystem, Constants.AlignToTag.CenterReef_AREA_REEF, Constants.AlignToTag.CenterReef_Y, Constants.AlignToTag.CenterReef_OMEGA),
                new CoralOutTakeCommand(coralSubsystem),
                new AlignWithTag(swervesubsystem, Constants.AlignToTag.CenterReef_AREA_STAGE, Constants.AlignToTag.CenterReef_Y, Constants.AlignToTag.CenterReef_OMEGA),
                new GoToBottomCommand(elevatorSubsystem)
        );
    }
}
