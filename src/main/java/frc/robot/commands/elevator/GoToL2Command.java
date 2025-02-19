
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class GoToL2Command extends Command {
   private final ElevatorSubsystem elevatorSubsystem;

   public GoToL2Command(ElevatorSubsystem elevatorSubsystem) {
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
      if (elevatorSubsystem.getPosition() <= Constants.ElevatorConstants.L2Position - 1)
      {
         elevatorSubsystem.setSpeed(Constants.ElevatorConstants.SPEED);
      }
      else if (elevatorSubsystem.getPosition() > Constants.ElevatorConstants.L2Position + 1)
      {
         elevatorSubsystem.setSpeed(-Constants.ElevatorConstants.SPEED);
      }
      else
      {
         elevatorSubsystem.stopElevator();
      }


   }

   @Override
   public boolean isFinished() {
      // TODO: Make this return true when this Command no longer needs to run execute()
      return false;
   }

   @Override
   public void end(boolean interrupted) {
      elevatorSubsystem.stopElevator();
   }
}
