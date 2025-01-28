package frc.robot.commands.dummycommands;


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
import frc.robot.subsystems.Swerve;

public class CoralIntakeCommand extends Command {

    private final Swerve drivetrainSubsystem;
    public CoralIntakeCommand(Swerve drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        new WaitCommand(2);
    }





    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

}
