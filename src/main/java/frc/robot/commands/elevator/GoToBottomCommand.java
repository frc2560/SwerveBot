package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class GoToBottomCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;

    public GoToBottomCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevatorSubsystem.setSpeed(-Constants.ElevatorConstants.SPEED);

    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getBottomSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
    }
}
