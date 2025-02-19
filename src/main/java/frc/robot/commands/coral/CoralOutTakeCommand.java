package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;


public class CoralOutTakeCommand extends Command {
    private final CoralIntakeSubsystem coralIntakeSubsystem;

    public CoralOutTakeCommand(CoralIntakeSubsystem coralIntakeSubsystem) {
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.coralIntakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralIntakeSubsystem.outtakeCoral();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return coralIntakeSubsystem.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        coralIntakeSubsystem.stopIntake();
    }
}
