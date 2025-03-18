package frc.robot.commands.scoreCoral;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.GoToL4Command;
import frc.robot.commands.movement.AlignWithTag;
import frc.robot.commands.movement.TurnToAngleCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class RightL4SetUpCommandGroup extends SequentialCommandGroup {
    public RightL4SetUpCommandGroup(ElevatorSubsystem elevatorSubsystem, Swerve swervesubsystem) {
        // TODO: Add your sequential commands in super()
        //       super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new TurnToAngleCommand(swervesubsystem),
                new ParallelCommandGroup(
                        new AlignWithTag(swervesubsystem, Constants.AlignToTag.RightReef_AREA_L4 , Constants.AlignToTag.RightReef_Y_STAGE, Constants.AlignToTag.RightReef_OMEGA_STAGE)
                ).withTimeout(3),
                new TurnToAngleCommand(swervesubsystem),
                new GoToL4Command(elevatorSubsystem)

        );
    }
}