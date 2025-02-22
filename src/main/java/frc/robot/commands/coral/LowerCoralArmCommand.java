package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class LowerCoralArmCommand extends Command {

   private final CoralSubsystem coralSubsystem;

   public LowerCoralArmCommand(CoralSubsystem coralSubsystem) {
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
      coralSubsystem.moveArmDown();

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