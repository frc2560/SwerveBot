package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;


public class LowerArmCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem;

    public LowerArmCommand(AlgaeSubsystem algaeSubsystem) {
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
        algaeSubsystem.armDown();

    }

    @Override
    public boolean isFinished() {
        return algaeSubsystem.isLowerLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stopArm();
    }
}
