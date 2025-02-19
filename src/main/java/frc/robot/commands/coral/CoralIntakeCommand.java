package frc.robot.commands.coral;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class CoralIntakeCommand extends Command {


    private final CoralSubsystem coralSubsystem;
    public CoralIntakeCommand(CoralSubsystem coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralSubsystem.intakeCoral();
    }

    @Override
    public boolean isFinished() {
      return coralSubsystem.hasCoral();
    }



    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stopIntake();
    }

}
