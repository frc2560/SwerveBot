package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class CoralOutTakeCommand extends Command {
    private final CoralSubsystem coralSubsystem;

    public CoralOutTakeCommand(CoralSubsystem coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.coralSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralSubsystem.outtakeCoral();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        //return !coralSubsystem.hasCoral();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stopIntake();
    }
}
