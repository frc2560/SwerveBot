package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;


public class OutTakeCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem;

    public OutTakeCommand(AlgaeSubsystem algaeSubsystem) {
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
        algaeSubsystem.outtakeAlgae();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return algaeSubsystem.hasAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stopIntake();
    }
}
