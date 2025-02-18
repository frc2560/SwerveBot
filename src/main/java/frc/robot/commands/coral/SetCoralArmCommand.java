package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class SetCoralArmCommand extends Command {
    private final CoralIntakeSubsystem coralIntakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public SetCoralArmCommand(CoralIntakeSubsystem coralIntakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.coralIntakeSubsystem, this.elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(coralIntakeSubsystem.getArmLocation() > Constants.Coral.IntakePosition + 1)
        {
            coralIntakeSubsystem.moveArmDown();
        }
        else if(coralIntakeSubsystem.getArmLocation() <=  Constants.Coral.IntakePosition - 1)
        {
            coralIntakeSubsystem.moveArmUp();
        }
        coralIntakeSubsystem.stopArm();

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        coralIntakeSubsystem.stopArm();
    }
}
