package frc.robot.commands.dummycommands;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class GrabAlgaeL1Command extends Command {

    private final Swerve drivetrainSubsystem;
    private final AlgaeSubsystem algaeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    public GrabAlgaeL1Command(Swerve drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.algaeSubsystem = algaeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(drivetrainSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(algaeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {


    }





    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

}
