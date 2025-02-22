package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;


public class SetCoralArmL1L2L3Command extends Command {

   private final CoralSubsystem coralSubsystem;

   public SetCoralArmL1L2L3Command(CoralSubsystem coralSubsystem) {
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
      if(coralSubsystem.getArmLocation() < Constants.Coral.OuttakeL1L2L3Position + 1)
      {
         coralSubsystem.moveArmDown();
      }
      else if(coralSubsystem.getArmLocation() >=  Constants.Coral.OuttakeL1L2L3Position - 1)
      {
         coralSubsystem.moveArmUp();
      }

   }

   @Override
   public boolean isFinished() {
      // TODO: Make this return true when this Command no longer needs to run execute()
      boolean test = coralSubsystem.getArmLocation() < Constants.Coral.OuttakeL1L2L3Position + 1;
      boolean test2 = coralSubsystem.getArmLocation() >=  Constants.Coral.OuttakeL1L2L3Position - 1;
      return test && test2;
   }

   @Override
   public void end(boolean interrupted) {
      coralSubsystem.stopArm();
   }
}
