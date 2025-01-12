package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class ChaseTagCommand extends Command {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

  private final Swerve drivetrainSubsystem;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);


  public ChaseTagCommand(
        Swerve drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.Sensor.LIMELIGHT, new int[]{TAG_TO_CHASE});
    LimelightHelpers.SetFiducialDownscalingOverride(Constants.Sensor.LIMELIGHT, 2.0f);
    var robotPose = drivetrainSubsystem.getPose();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = drivetrainSubsystem.getPose();
    var robotPose =
            new Pose3d(
                    robotPose2d.getX(),
                    robotPose2d.getY(),
                    0.0,
                    new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    //Check if Limelight is seeing a target
    if (LimelightHelpers.getTV(Constants.Sensor.LIMELIGHT)) {
      // What are the current target's pose in relation to the robot?
      double tx = LimelightHelpers.getTX(Constants.Sensor.LIMELIGHT);  // Horizontal offset from crosshair to target in degrees
      double ty = LimelightHelpers.getTY(Constants.Sensor.LIMELIGHT);  // Vertical offset from crosshair to target in degrees
      double ta = LimelightHelpers.getTA(Constants.Sensor.LIMELIGHT);

      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

      var aprilTagToDriveTo = aprilTagFieldLayout.getTagPose(TAG_TO_CHASE);


      // Set the goals for the controllers
      if(aprilTagToDriveTo.isPresent())
      {
        //Set how far from Tag we want to be
        var targetOffset = new Translation2d(-1, 0);
        //Where is the tag
        var pose2d = aprilTagToDriveTo.get().toPose2d();
        //Set the goal as the difference between the tag and offset.
        var targetPose = new Pose2d(pose2d.getTranslation().minus(targetOffset), pose2d.getRotation());
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        omegaController.setGoal(targetPose.getRotation().getRadians());
      }

    }


    if (!LimelightHelpers.getTV(Constants.Sensor.LIMELIGHT)) {
      // No target has been visible
      drivetrainSubsystem.stop();
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      drivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
              omegaSpeed * Constants.Swerve.maxAngularVelocity,
              true,
              true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
