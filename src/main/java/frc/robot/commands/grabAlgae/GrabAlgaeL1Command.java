package frc.robot.commands.grabAlgae;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.algae.AlgaeIntakeCommand;
import frc.robot.commands.algae.LowerAlgaeArmCommand;
import frc.robot.commands.elevator.GoToBottomCommand;
import frc.robot.commands.elevator.GoToL1AlgaeCommand;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class GrabAlgaeL1Command extends SequentialCommandGroup {
    public GrabAlgaeL1Command(ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem, Swerve swerveSubsystem) {
        super(  new AlignWithTag(swerveSubsystem, Constants.AlignToTag.CenterReef_AREA_STAGE, Constants.AlignToTag.CenterReef_Y_REEF, Constants.AlignToTag.CenterReef_OMEGA_REEF),
                new LowerAlgaeArmCommand(algaeSubsystem),
                new GoToL1AlgaeCommand(elevatorSubsystem),
                new AlignWithTag(swerveSubsystem, Constants.AlignToTag.CenterReef_AREA_REEF, Constants.AlignToTag.CenterReef_Y_REEF, Constants.AlignToTag.CenterReef_OMEGA_REEF),
                new AlgaeIntakeCommand(algaeSubsystem),
                new AlignWithTag(swerveSubsystem, Constants.AlignToTag.CenterReef_AREA_STAGE, Constants.AlignToTag.CenterReef_Y_REEF, Constants.AlignToTag.CenterReef_OMEGA_REEF),
                new GoToBottomCommand(elevatorSubsystem));
    }
}