package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;


public class SetAlgaeArmUpperCommand extends Command {


    private final AlgaeSubsystem algaeSubsystem;

    public SetAlgaeArmUpperCommand(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.algaeSubsystem);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(algaeSubsystem.getArmPosition()< Constants.Algae.UpperArmPosition + 1)
        {
            algaeSubsystem.armDown();
        }
        else if(algaeSubsystem.getArmPosition() >=  Constants.Algae.UpperArmPosition- 1)
        {
            algaeSubsystem.armUp();
        }


    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        boolean test = algaeSubsystem.getArmPosition()< Constants.Algae.UpperArmPosition + 1;
        boolean test2 = algaeSubsystem.getArmPosition() >=  Constants.Algae.UpperArmPosition - 1;
        return test && test2;
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stopArm();

    }
}
