package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;


public class IntakeCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem;

    public IntakeCommand(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.algaeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        algaeSubsystem.intakeAlgae();
    }

    @Override
    public boolean isFinished() {
        return algaeSubsystem.hasAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stopIntake();
    }
}
