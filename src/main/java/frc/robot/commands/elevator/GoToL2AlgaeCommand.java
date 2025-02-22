
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class GoToL2AlgaeCommand extends Command {
   private final ElevatorSubsystem elevatorSubsystem;

   public GoToL2AlgaeCommand(ElevatorSubsystem elevatorSubsystem) {
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
      if (elevatorSubsystem.getPosition() <= Constants.ElevatorConstants.L2AlgaePosition - 1)
      {
         elevatorSubsystem.setSpeed(Constants.ElevatorConstants.UPSPEED);
      }
      else if (elevatorSubsystem.getPosition() > Constants.ElevatorConstants.L2AlgaePosition + 1)
      {
         elevatorSubsystem.setSpeed(-Constants.ElevatorConstants.DOWNSPEED);
      }

   }

   @Override
   public boolean isFinished() {
      // TODO: Make this return true when this Command no longer needs to run execute()
      boolean test = elevatorSubsystem.getPosition() > Constants.ElevatorConstants.L2AlgaePosition - 1;
      boolean test2 = elevatorSubsystem.getPosition() <= Constants.ElevatorConstants.L2AlgaePosition + 1;
      return test && test2;
   }

   @Override
   public void end(boolean interrupted)
   {
      elevatorSubsystem.stopElevator();
   }
}
