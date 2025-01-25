package frc.robot.commands;

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

public class ChaseTagCommand extends Command {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(0.25, 1);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(0.25, 1);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1);
  
  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

  private final Swerve drivetrainSubsystem;

  private final PIDController xController = new PIDController(0.1, 0, 0);
  private final PIDController yController = new PIDController(0.1, 0, 0);
  private final PIDController omegaController = new PIDController(0.1, 0, 0);


  public ChaseTagCommand(
        Swerve drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    xController.setTolerance(1);
    yController.setTolerance(10);
    omegaController.setTolerance(10);
    //omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.Sensor.LIMELIGHT, new int[]{TAG_TO_CHASE});
    LimelightHelpers.SetFiducialDownscalingOverride(Constants.Sensor.LIMELIGHT, 2.0f);
    xController.setSetpoint(5);
    yController.setSetpoint(0);
  }

  @Override
  public void execute() {
    //Check if Limelight is seeing a target


    if (!LimelightHelpers.getTV(Constants.Sensor.LIMELIGHT)) {
      // No target has been visible
      drivetrainSubsystem.stop();
    }
    else
    {
      double tx = LimelightHelpers.getTX(Constants.Sensor.LIMELIGHT);  // Horizontal offset from crosshair to target in degrees
      double ty = LimelightHelpers.getTY(Constants.Sensor.LIMELIGHT);  // Vertical offset from crosshair to target in degrees
      double ta = LimelightHelpers.getTA(Constants.Sensor.LIMELIGHT);

      // Drive to the target
      var xSpeed = MathUtil.clamp(xController.calculate(ta, 5), -0.05, .05);
      if (xController.atSetpoint()) {
        xSpeed = 0;
      }

      var ySpeed =  MathUtil.clamp(yController.calculate(ty, 0), -0.05, .05);
      if (yController.atSetpoint()) {
        ySpeed = 0;
      }

      var omegaSpeed = MathUtil.clamp(omegaController.calculate(tx, 0), -0.04, 0.04);
      if (omegaController.atSetpoint()) {
        omegaSpeed = 0;
      }

      drivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
              omegaSpeed,
              true,
              true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
