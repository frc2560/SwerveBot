package frc.robot.commands.scoreCoral;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.GoToL4Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.commands.movement.TurnToAngleCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class LeftL4SetUpCommandGroup extends SequentialCommandGroup {
    public LeftL4SetUpCommandGroup(ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem, boolean isAuto) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new TurnToAngleCommand(swervesubsystem, isAuto),
                new ParallelCommandGroup(
                        new AlignWithTag(swervesubsystem, Constants.AlignToTag.LeftReef_AREA_L4, Constants.AlignToTag.LeftReef_Y_STAGE, Constants.AlignToTag.LeftReef_OMEGA_STAGE)
                ).withTimeout(3),
                new TurnToAngleCommand(swervesubsystem, isAuto),
                new GoToL4Command(elevatorSubsystem)


        );
    }
}