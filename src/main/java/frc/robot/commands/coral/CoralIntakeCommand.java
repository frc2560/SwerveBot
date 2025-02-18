package frc.robot.commands.coral;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

public class CoralIntakeCommand extends Command {

    private final Swerve drivetrainSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralIntakeSubsystem coralIntakeSubsystem;
    public CoralIntakeCommand(Swerve drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, CoralIntakeSubsystem coralIntakeSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralIntakeSubsystem = coralIntakeSubsystem;

        addRequirements(drivetrainSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(coralIntakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralIntakeSubsystem.intakeCoral();
    }

    @Override
    public boolean isFinished() {
      return coralIntakeSubsystem.hasCoral();
    }



    @Override
    public void end(boolean interrupted) {
        coralIntakeSubsystem.stopIntake();
    }

}
