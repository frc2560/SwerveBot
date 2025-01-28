package frc.robot.commands.dummycommands;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class GrabAlgaeL1Command extends Command {

    private final Swerve drivetrainSubsystem;
    public GrabAlgaeL1Command(Swerve drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        new WaitCommand(2);
    }





    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

}
