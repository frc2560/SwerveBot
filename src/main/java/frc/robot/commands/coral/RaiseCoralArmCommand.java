package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class RaiseCoralArmCommand extends Command {

   private final CoralSubsystem coralSubsystem;

   public RaiseCoralArmCommand(CoralSubsystem coralSubsystem) {
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
    coralSubsystem.moveArmUp();

   }

   @Override
   public boolean isFinished() {
      return coralSubsystem.isUpperSwitchPressed();
   }

   @Override
   public void end(boolean interrupted) {
      coralSubsystem.stopArm();
   }
}