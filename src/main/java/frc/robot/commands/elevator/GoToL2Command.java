
package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class GoToL2Command extends Command {
   private final ElevatorSubsystem elevatorSubsystem;
   private final PIDController elevatorController = new PIDController(0.05, 0, 0 );

   public GoToL2Command(ElevatorSubsystem elevatorSubsystem) {
      this.elevatorSubsystem = elevatorSubsystem;
      // each subsystem used by the command must be passed into the
      // addRequirements() method (which takes a vararg of Subsystem)
      addRequirements(this.elevatorSubsystem);
   }

   @Override
   public void initialize() {
      elevatorController.setTolerance(0.1);

   }

   @Override
   public void execute() {
      var elevatorSpeed = MathUtil.clamp(elevatorController.calculate(elevatorSubsystem.getPosition(), Constants.ElevatorConstants.L2Position), -Constants.ElevatorConstants.DOWNSPEED , Constants.ElevatorConstants.UPSPEED);
      if (elevatorController.atSetpoint()){
         elevatorSpeed = 0;
      }
      elevatorSubsystem.setSpeed(elevatorSpeed);
   }



   @Override
   public boolean isFinished() {
      // TODO: Make this return true when this Command no longer needs to run execute()
      return elevatorController.atSetpoint();
   }

   @Override
   public void end(boolean interrupted)
   {
      elevatorSubsystem.stopElevator();
   }
}
