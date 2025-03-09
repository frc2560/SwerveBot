package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


public class LowerClimberArmCommand extends Command {

    private final ClimberSubsystem climberSubsystem;

    public LowerClimberArmCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climberSubsystem.climberArmDown();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopArm();

    }
}
