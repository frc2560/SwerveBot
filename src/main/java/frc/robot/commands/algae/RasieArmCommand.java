package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;


public class RasieArmCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem;

    public RasieArmCommand(AlgaeSubsystem algaeSubsystem) {
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
        algaeSubsystem.armUp();

    }

    @Override
    public boolean isFinished() {
        if(algaeSubsystem.getArmPosition() > Constants.Algae.UpperArmPosition)
        {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stopArm();

    }
}
